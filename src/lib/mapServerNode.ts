import type { CarState, Obstacle } from './appModel'
import { KNOWN_OBSTACLE_COORDINATES } from './generatedMapCoordinates'

type CollisionCheckFn = (state: CarState, obstacleCoordinates: Float64Array) => Promise<boolean>

export type MapBoundingBox = {
  minX: number
  minY: number
  maxX: number
  maxY: number
}

export type MapServerConfig = {
  backToCenter: number
  scanRadius: number
}

export type MapServerSnapshot = {
  boundingBox: MapBoundingBox
  knownObstacles: Obstacle[]
  unknownObstacles: Obstacle[]
}

export type MapServerUpdateResult = MapServerSnapshot & {
  newObstacles: Obstacle[]
}

const MAP_WIDTH = 60
const MAP_HEIGHT = 60
const UNKNOWN_OBSTACLE_COUNT = 40

function createKnownObstacleTemplate(): Obstacle[] {
  const obstacles: Obstacle[] = []
  for (let index = 0; index < KNOWN_OBSTACLE_COORDINATES.length; index += 2) {
    obstacles.push({
      id: `k-${index / 2}`,
      x: KNOWN_OBSTACLE_COORDINATES[index],
      y: KNOWN_OBSTACLE_COORDINATES[index + 1],
    })
  }
  return obstacles
}

export function flattenObstacleCoordinates(obstacles: Obstacle[]) {
  return Float64Array.from(obstacles.flatMap((obstacle) => [obstacle.x, obstacle.y]))
}

function buildBoundingBox(obstacles: Obstacle[]): MapBoundingBox {
  return obstacles.reduce<MapBoundingBox>(
    (bounds, obstacle) => ({
      minX: Math.min(bounds.minX, obstacle.x),
      minY: Math.min(bounds.minY, obstacle.y),
      maxX: Math.max(bounds.maxX, obstacle.x),
      maxY: Math.max(bounds.maxY, obstacle.y),
    }),
    {
      minX: Number.POSITIVE_INFINITY,
      minY: Number.POSITIVE_INFINITY,
      maxX: Number.NEGATIVE_INFINITY,
      maxY: Number.NEGATIVE_INFINITY,
    },
  )
}

function createUnknownObstacles(bounds: MapBoundingBox) {
  const obstacles: Obstacle[] = []
  while (obstacles.length < UNKNOWN_OBSTACLE_COUNT) {
    obstacles.push({
      id: `u-${obstacles.length}`,
      x: bounds.minX + Math.random() * (bounds.maxX - bounds.minX),
      y: bounds.minY + Math.random() * (bounds.maxY - bounds.minY),
    })
  }
  return obstacles
}

export class MapServerNode {
  private knownObstacles: Obstacle[] = []
  private unknownObstacles: Obstacle[] = []
  private boundingBox: MapBoundingBox = { minX: 0, minY: 0, maxX: 0, maxY: 0 }
  private undiscoveredUnknownObstacleIds = new Set<string>()

  constructor(
    private readonly checkCollision: CollisionCheckFn,
    private readonly config: MapServerConfig,
  ) {}

  setConfig(config: MapServerConfig) {
    this.config.backToCenter = config.backToCenter
    this.config.scanRadius = config.scanRadius
  }

  init(): MapServerSnapshot {
    this.knownObstacles = createKnownObstacleTemplate()
    this.boundingBox = buildBoundingBox(this.knownObstacles)
    this.unknownObstacles = createUnknownObstacles(this.boundingBox)
    this.undiscoveredUnknownObstacleIds = new Set(
      this.unknownObstacles.map((obstacle) => obstacle.id),
    )
    return this.getSnapshot()
  }

  getSnapshot(): MapServerSnapshot {
    return {
      boundingBox: { ...this.boundingBox },
      knownObstacles: this.knownObstacles.slice(),
      unknownObstacles: this.unknownObstacles.slice(),
    }
  }

  async generateRandomInitialState(): Promise<CarState> {
    const obstacleCoordinates = flattenObstacleCoordinates([
      ...this.knownObstacles,
      ...this.unknownObstacles,
    ])
    while (true) {
      const candidate: CarState = {
        x: Math.random() * MAP_WIDTH,
        y: Math.random() * MAP_HEIGHT,
        yaw: Math.random() * Math.PI * 2 - Math.PI,
        velocity: 0,
        steer: 0,
      }
      if (!(await this.checkCollision(candidate, obstacleCoordinates))) {
        return candidate
      }
    }
  }

  update(state: Pick<CarState, 'x' | 'y' | 'yaw'>): MapServerUpdateResult | null {
    const scanCenterX = state.x + this.config.backToCenter * Math.cos(state.yaw)
    const scanCenterY = state.y + this.config.backToCenter * Math.sin(state.yaw)
    const newObstacles = this.unknownObstacles.filter(
      (obstacle) =>
        this.undiscoveredUnknownObstacleIds.has(obstacle.id) &&
        Math.hypot(obstacle.x - scanCenterX, obstacle.y - scanCenterY) <= this.config.scanRadius,
    )

    if (newObstacles.length === 0) {
      return null
    }

    for (const obstacle of newObstacles) {
      this.undiscoveredUnknownObstacleIds.delete(obstacle.id)
    }
    this.knownObstacles = [...this.knownObstacles, ...newObstacles]

    return {
      ...this.getSnapshot(),
      newObstacles,
    }
  }
}

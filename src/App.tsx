import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react'

import { HistoryChart } from './components/HistoryChart'
import { MapViewport } from './components/MapViewport'
import {
  CAR_CONSTANTS,
  HISTORY_LIMIT,
  type CarState,
  type Mode,
  type Obstacle,
} from './lib/appModel'
import { MapServerNode, flattenObstacleCoordinates, type MapBoundingBox } from './lib/mapServerNode'
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  checkCollision,
  checkTrajectoryCollision,
  ensureWasmCore,
  getCarConfigSnapshot,
  initSimulation,
  resetComputeWorker,
  resumeSimulationMotion,
  setHybridAStarProgressListener,
  setLocalPlannerState,
  setLocalPlannerTrajectory,
  setLocalPlannerUpdateListener,
  setSimulationState,
  setSimulationStateListener,
  solveHybridAStar,
  stopSimulation,
  stopSimulationMotion,
  type HybridAStarProgress,
  type HybridAStarStartSeedPoint,
  type LocalPlannerPathPoint,
  type LocalPlannerReferencePoint,
  type LocalPlannerTrajectoryPoint,
  type WasmConfigSnapshot,
} from './lib/wasmCore'
import { TrajectoryCollisionCheckingNode } from './lib/trajectoryCollisionCheckingNode'

const LOCAL_PLANNER_UPDATE_INTERVAL_MS = 100
const LOCAL_PLANNER_DT = 0.07
const REPLAN_MAX_SPEED = 5 / 3.6
const MAX_GLOBAL_PLANNER_DISPLAY_BATCHES = 32

export type CarShape = {
  wheelBase: number
  length: number
  width: number
  backToWheel: number
  wheelLength: number
  wheelWidth: number
  wheelSpacing: number
  backToCenter: number
}

export type MotionLimits = {
  scanRadius: number
  maxSpeedKmh: number
  minSpeedKmh: number
  maxSteerDeg: number
}

export type GoalUnreachableState = {
  visible: boolean
  x: number
  y: number
}

type DragStartState = {
  startX: number
  startY: number
}

type MapServerSnapshot = {
  boundingBox: MapBoundingBox
  knownObstacles: Obstacle[]
  unknownObstacles: Obstacle[]
}

type DashboardLayout = 'split' | 'stacked'

const FALLBACK_MAP_BOUNDING_BOX: MapBoundingBox = {
  minX: 0,
  minY: 0,
  maxX: 80,
  maxY: 60,
}

const DEFAULT_CAR_SHAPE: CarShape = {
  wheelBase: CAR_CONSTANTS.wheelBase,
  length: CAR_CONSTANTS.length,
  width: CAR_CONSTANTS.width,
  backToWheel: CAR_CONSTANTS.backToWheel,
  wheelLength: CAR_CONSTANTS.wheelLength,
  wheelWidth: CAR_CONSTANTS.wheelWidth,
  wheelSpacing: CAR_CONSTANTS.wheelSpacing,
  backToCenter: CAR_CONSTANTS.backToCenter,
}

const DEFAULT_MOTION_LIMITS: MotionLimits = {
  scanRadius: CAR_CONSTANTS.scanRadius,
  maxSpeedKmh: CAR_CONSTANTS.maxSpeed * 3.6,
  minSpeedKmh: CAR_CONSTANTS.minSpeed * 3.6,
  maxSteerDeg: (CAR_CONSTANTS.maxSteer * 180) / Math.PI,
}

function createCarShape(snapshot: WasmConfigSnapshot): CarShape {
  return {
    wheelBase: snapshot.wheelBase,
    length: snapshot.length,
    width: snapshot.width,
    backToWheel: snapshot.backToWheel,
    wheelLength: snapshot.wheelLength,
    wheelWidth: snapshot.wheelWidth,
    wheelSpacing: snapshot.wheelSpacing,
    backToCenter: snapshot.backToCenter,
  }
}

function createMotionLimits(snapshot: WasmConfigSnapshot): MotionLimits {
  return {
    scanRadius: snapshot.scanRadius,
    maxSpeedKmh: snapshot.maxSpeed * 3.6,
    minSpeedKmh: snapshot.minSpeed * 3.6,
    maxSteerDeg: (snapshot.maxSteer * 180) / Math.PI,
  }
}

function isPointInBounds(point: { x: number; y: number }, bounds: MapBoundingBox) {
  return point.x >= bounds.minX && point.x <= bounds.maxX && point.y >= bounds.minY && point.y <= bounds.maxY
}

function toTrajectoryPath(points: LocalPlannerTrajectoryPoint[]) {
  return points.map((point) => ({ x: point.x, y: point.y }))
}

function toHybridAStarStartSeed(points: LocalPlannerReferencePoint[]): HybridAStarStartSeedPoint[] {
  return points.map((point) => ({
    x: point.x,
    y: point.y,
    yaw: point.yaw,
    velocity: point.velocity,
  }))
}

type AutoShrinkHeadingProps = {
  text: string
}

const AUTO_HEADING_MAX_FONT_SIZE_PX = 28
const AUTO_HEADING_MIN_FONT_SIZE_PX = 11
const STACKED_LAYOUT_MAX_WIDTH_PX = 560
const STACKED_LAYOUT_MIN_MAP_HEIGHT_PX = 220
const STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX = 110
const STACKED_LAYOUT_GAP_PX = 8

function clamp(value: number, minValue: number, maxValue: number) {
  return Math.min(maxValue, Math.max(minValue, value))
}

function formatFixedWithoutNegativeZero(value: number, digits: number) {
  const roundedValue = Number(value.toFixed(digits))
  return (Object.is(roundedValue, -0) ? 0 : roundedValue).toFixed(digits)
}

function AutoShrinkHeading({ text }: AutoShrinkHeadingProps) {
  const headingRef = useRef<HTMLHeadingElement | null>(null)
  const textRef = useRef<HTMLSpanElement | null>(null)

  useLayoutEffect(() => {
    const heading = headingRef.current
    const label = textRef.current
    if (!heading || !label) {
      return
    }

    let frame = 0

    const updateFontSize = () => {
      frame = 0

      const availableWidth = heading.clientWidth
      if (availableWidth <= 0) {
        return
      }

      label.style.fontSize = `${AUTO_HEADING_MAX_FONT_SIZE_PX}px`
      const naturalWidth = label.scrollWidth
      if (naturalWidth <= 0) {
        return
      }

      const fittedSize = clamp(
        (AUTO_HEADING_MAX_FONT_SIZE_PX * availableWidth) / naturalWidth,
        AUTO_HEADING_MIN_FONT_SIZE_PX,
        AUTO_HEADING_MAX_FONT_SIZE_PX,
      )
      label.style.fontSize = `${fittedSize}px`
    }

    const scheduleUpdate = () => {
      if (frame !== 0) {
        return
      }
      frame = window.requestAnimationFrame(updateFontSize)
    }

    updateFontSize()

    const resizeObserver = new ResizeObserver(scheduleUpdate)
    resizeObserver.observe(heading)

    return () => {
      resizeObserver.disconnect()
      if (frame !== 0) {
        window.cancelAnimationFrame(frame)
      }
    }
  }, [text])

  return (
    <h2 ref={headingRef} className="auto-shrink-heading">
      <span ref={textRef} className="auto-shrink-heading__text">
        {text}
      </span>
    </h2>
  )
}

function App() {
  const [mode, setMode] = useState<Mode>('goal')
  const [timestamp, setTimestamp] = useState(0)
  const [mapSnapshot, setMapSnapshot] = useState<MapServerSnapshot>({
    boundingBox: FALLBACK_MAP_BOUNDING_BOX,
    knownObstacles: [],
    unknownObstacles: [],
  })
  const [carShape, setCarShape] = useState<CarShape>(DEFAULT_CAR_SHAPE)
  const [motionLimits, setMotionLimits] = useState<MotionLimits>(DEFAULT_MOTION_LIMITS)
  const [car, setCar] = useState<CarState | null>(null)
  const [goal, setGoal] = useState<CarState | null>(null)
  const [pressedPose, setPressedPose] = useState<CarState | null>(null)
  const [goalUnreachable, setGoalUnreachable] = useState<GoalUnreachableState>({ visible: false, x: 0, y: 0 })
  const [globalTrajectory, setGlobalTrajectory] = useState<LocalPlannerTrajectoryPoint[] | null>(null)
  const [localTrajectory, setLocalTrajectory] = useState<LocalPlannerPathPoint[]>([])
  const [referencePoints, setReferencePoints] = useState<LocalPlannerReferencePoint[]>([])
  const [globalPlannerSegments, setGlobalPlannerSegments] = useState<HybridAStarProgress['segments'][]>([])
  const [velocityHistory, setVelocityHistory] = useState([{ t: 0, value: 0 }])
  const [steerHistory, setSteerHistory] = useState([{ t: 0, value: 0 }])
  const [dashboardLayout, setDashboardLayout] = useState<DashboardLayout>('split')

  const mapServerNodeRef = useRef<MapServerNode | null>(null)
  const carRef = useRef<CarState | null>(car)
  const timestampRef = useRef(timestamp)
  const goalRef = useRef<CarState | null>(goal)
  const mapSnapshotRef = useRef(mapSnapshot)
  const globalTrajectoryRef = useRef<LocalPlannerTrajectoryPoint[] | null>(globalTrajectory)
  const localPlanningRef = useRef(false)
  const brakeTrajectoryRef = useRef<LocalPlannerReferencePoint[] | null>(null)
  const planningRequestRef = useRef(0)
  const dragStartRef = useRef<DragStartState | null>(null)
  const trajectoryCollisionCheckingNodeRef = useRef<TrajectoryCollisionCheckingNode | null>(null)
  const dashboardGridRef = useRef<HTMLElement | null>(null)

  if (trajectoryCollisionCheckingNodeRef.current === null) {
    trajectoryCollisionCheckingNodeRef.current = new TrajectoryCollisionCheckingNode(checkTrajectoryCollision)
  }

  if (mapServerNodeRef.current === null) {
    mapServerNodeRef.current = new MapServerNode(checkCollision, {
      backToCenter: DEFAULT_CAR_SHAPE.backToCenter,
      scanRadius: DEFAULT_MOTION_LIMITS.scanRadius,
    })
  }

  useEffect(() => {
    carRef.current = car
  }, [car])

  useEffect(() => {
    timestampRef.current = timestamp
  }, [timestamp])

  useEffect(() => {
    goalRef.current = goal
  }, [goal])

  useEffect(() => {
    mapSnapshotRef.current = mapSnapshot
  }, [mapSnapshot])

  useEffect(() => {
    globalTrajectoryRef.current = globalTrajectory
  }, [globalTrajectory])

  useLayoutEffect(() => {
    const host = dashboardGridRef.current
    if (!host) {
      return
    }

    const updateLayout = () => {
      const width = host.clientWidth
      const height = host.clientHeight
      const canStack =
        width <= STACKED_LAYOUT_MAX_WIDTH_PX &&
        height >= STACKED_LAYOUT_MIN_MAP_HEIGHT_PX + STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX + STACKED_LAYOUT_GAP_PX

      setDashboardLayout(canStack ? 'stacked' : 'split')
    }

    updateLayout()

    const resizeObserver = new ResizeObserver(updateLayout)
    resizeObserver.observe(host)

    return () => {
      resizeObserver.disconnect()
    }
  }, [])

  useEffect(() => {
    let active = true

    void ensureWasmCore()
      .then(() => getCarConfigSnapshot())
      .then((snapshot) => {
        if (!active) {
          return
        }

        const nextCarShape = createCarShape(snapshot)
        const nextMotionLimits = createMotionLimits(snapshot)
        setCarShape(nextCarShape)
        setMotionLimits(nextMotionLimits)
        mapServerNodeRef.current?.setConfig({
          backToCenter: snapshot.backToCenter,
          scanRadius: snapshot.scanRadius,
        })
      })
      .catch((error) => {
        console.error('Failed to initialize WASM core', error)
      })

    return () => {
      active = false
    }
  }, [])

  useEffect(() => {
    setHybridAStarProgressListener((progress) => {
      if (progress.token !== planningRequestRef.current) {
        return
      }
      setGlobalPlannerSegments((segments) => {
        const nextSegments = [...segments, progress.segments]
        return nextSegments.length > MAX_GLOBAL_PLANNER_DISPLAY_BATCHES
          ? nextSegments.slice(-MAX_GLOBAL_PLANNER_DISPLAY_BATCHES)
          : nextSegments
      })
    })
    return () => setHybridAStarProgressListener(null)
  }, [])

  useEffect(() => {
    let active = true

    setLocalPlannerUpdateListener((result) => {
      if (!active || !localPlanningRef.current) {
        return
      }

      brakeTrajectoryRef.current = result.brakeTrajectory
      setLocalTrajectory(result.localTrajectory)
      setReferencePoints(result.referencePoints)
    })

    setSimulationStateListener((event) => {
      if (!active) {
        return
      }

      void setLocalPlannerState(event.state, event.timestamp, LOCAL_PLANNER_DT, LOCAL_PLANNER_UPDATE_INTERVAL_MS).catch((error) => {
        console.error('Failed to update local planner state', error)
      })

      carRef.current = event.state
      timestampRef.current = event.timestamp
      setTimestamp(event.timestamp)
      setCar(event.state)
    })

    void (async () => {
      try {
        const mapServerNode = mapServerNodeRef.current
        if (!mapServerNode) {
          return
        }

        const snapshot = mapServerNode.init()
        if (!active) {
          return
        }
        trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(flattenObstacleCoordinates(snapshot.knownObstacles))
        setMapSnapshot(snapshot)

        const initialCar = await mapServerNode.generateRandomInitialState()
        if (!active) {
          return
        }

        carRef.current = initialCar
        timestampRef.current = 0
        setCar(initialCar)
        setTimestamp(0)

        await initSimulation(initialCar, 0)
      } catch (error) {
        console.error('Failed to initialize app state', error)
      }
    })()

    return () => {
      active = false
      setLocalPlannerUpdateListener(null)
      setSimulationStateListener(null)
      void stopSimulation().catch(() => { })
      resetComputeWorker('App unmounted')
    }
  }, [])

  useEffect(() => {
    if (!car) {
      return
    }

    const mapUpdate = mapServerNodeRef.current?.update(car)
    if (!mapUpdate || mapUpdate.newObstacles.length === 0) {
      return
    }

    mapSnapshotRef.current = mapUpdate
    trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(flattenObstacleCoordinates(mapUpdate.knownObstacles))
    setMapSnapshot(mapUpdate)

    void trajectoryCollisionCheckingNodeRef.current
      ?.checkCollision(flattenObstacleCoordinates(mapUpdate.newObstacles))
      .then((collided) => {
        if (!collided) {
          return
        }
      })
      .catch((error) => {
        console.error('Failed to check trajectory collision', error)
      })
  }, [car])

  useEffect(() => {
    if (!car) {
      return
    }

    setVelocityHistory((history) => [...history.slice(-HISTORY_LIMIT + 1), { t: timestamp, value: car.velocity * 3.6 }])
    setSteerHistory((history) => [
      ...history.slice(-HISTORY_LIMIT + 1),
      { t: timestamp, value: (car.steer * 180) / Math.PI },
    ])
  }, [car, timestamp])

  const clearGlobalPlannerDisplaySegments = useCallback(() => {
    setGlobalPlannerSegments([])
  }, [])

  const handleCancel = useCallback(async () => {
    planningRequestRef.current += 1
    dragStartRef.current = null
    setPressedPose(null)
    setGoalUnreachable((current) => ({ ...current, visible: false }))
    clearGlobalPlannerDisplaySegments()
    setLocalTrajectory([])
    setReferencePoints([])
    localPlanningRef.current = false
    brakeTrajectoryRef.current = null
    trajectoryCollisionCheckingNodeRef.current?.cancel()
    try {
      await Promise.all([stopSimulationMotion(), cancelHybridAStar(), cancelLocalPlanner()])
    } catch (error) {
      console.error('Failed to cancel current execution', error)
    }
  }, [clearGlobalPlannerDisplaySegments])

  const handleBrake = useCallback(async () => {
    planningRequestRef.current += 1
    setGoalUnreachable((current) => ({ ...current, visible: false }))
    clearGlobalPlannerDisplaySegments()
    trajectoryCollisionCheckingNodeRef.current?.cancel()
    try {
      await Promise.all([cancelHybridAStar(), brakeLocalPlanner()])
    } catch (error) {
      console.error('Failed to brake current execution', error)
    }
  }, [clearGlobalPlannerDisplaySegments])

  const runGlobalPlan = useCallback(async () => {
    const measuredState = carRef.current
    const goalState = goalRef.current

    if (!measuredState || !goalState) {
      return
    }

    const start =
      Math.abs(measuredState.velocity) > REPLAN_MAX_SPEED && brakeTrajectoryRef.current
        ? toHybridAStarStartSeed(brakeTrajectoryRef.current)
        : measuredState

    const requestId = planningRequestRef.current + 1
    planningRequestRef.current = requestId

    clearGlobalPlannerDisplaySegments()

    try {
      const result = await solveHybridAStar(
        start,
        goalState,
        flattenObstacleCoordinates(mapSnapshotRef.current.knownObstacles),
        4000,
        requestId,
      )
      if (planningRequestRef.current !== requestId) {
        return
      }

      clearGlobalPlannerDisplaySegments()
      if (!result) {
        setGlobalTrajectory(null)
        globalTrajectoryRef.current = null
        localPlanningRef.current = false
        const collisionChecker = trajectoryCollisionCheckingNodeRef.current
        await Promise.all([
          collisionChecker ? collisionChecker.setTrajectory(null) : Promise.resolve(false),
          setLocalPlannerTrajectory(null),
        ])
        setGoalUnreachable((current) => ({ ...current, visible: true }))
        return
      }
      if (result.token !== requestId) {
        return
      }
      const trajectory = result.path.map((point, index) => ({
        x: point.x,
        y: point.y,
        yaw: point.yaw,
        direction: result.directions[index] ?? 0,
      }))
      setGlobalTrajectory(trajectory)
      globalTrajectoryRef.current = trajectory
      localPlanningRef.current = true
      setGoalUnreachable((current) => ({ ...current, visible: false }))

      if (planningRequestRef.current !== requestId) {
        return
      }
      await setLocalPlannerTrajectory(trajectory)
      if (planningRequestRef.current !== requestId) {
        return
      }
      const collided = await trajectoryCollisionCheckingNodeRef.current!.setTrajectory(
        trajectory.map((point) => ({ x: point.x, y: point.y, yaw: point.yaw })),
      )
      if (planningRequestRef.current !== requestId) {
        return
      }
      if (collided) {
        return
      }

      await resumeSimulationMotion()
    } catch (error) {
      if (planningRequestRef.current !== requestId) {
        return
      }
      if (error instanceof Error && error.message === 'Hybrid A* search cancelled') {
        return
      }
      clearGlobalPlannerDisplaySegments()
      setGlobalTrajectory(null)
      globalTrajectoryRef.current = null
      const collisionChecker = trajectoryCollisionCheckingNodeRef.current
      await Promise.all([
        collisionChecker ? collisionChecker.setTrajectory(null) : Promise.resolve(false),
        setLocalPlannerTrajectory(null),
      ])
      setGoalUnreachable((current) => ({ ...current, visible: true }))
      console.error('Failed to compute global plan', error)
    }
  }, [clearGlobalPlannerDisplaySegments])

  const handleTrajectoryCollided = useCallback(async () => {
    setGlobalTrajectory(null)
    globalTrajectoryRef.current = null
    await runGlobalPlan()
  }, [runGlobalPlan])

  useEffect(() => {
    const node = trajectoryCollisionCheckingNodeRef.current
    if (!node) {
      return
    }

    node.setCollidedListener(() => {
      void brakeLocalPlanner().catch(() => { })
      void handleTrajectoryCollided().catch((error) => {
        console.error('Failed to handle trajectory collision', error)
      })
    })

    return () => {
      node.setCollidedListener(null)
    }
  }, [handleTrajectoryCollided])

  const handleRestart = useCallback(async () => {
    await handleCancel()
    goalRef.current = null
    globalTrajectoryRef.current = null
    brakeTrajectoryRef.current = null
    setGoal(null)
    setPressedPose(null)
    setGoalUnreachable((current) => ({ ...current, visible: false }))
    setGlobalTrajectory(null)

    try {
      const mapServerNode = mapServerNodeRef.current
      if (!mapServerNode) {
        return
      }

      const nextSnapshot = mapServerNode.init()
      mapSnapshotRef.current = nextSnapshot
      trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(flattenObstacleCoordinates(nextSnapshot.knownObstacles))
      setMapSnapshot(nextSnapshot)

      const nextCar = await mapServerNode.generateRandomInitialState()
      carRef.current = nextCar
      setCar(nextCar)
      await setSimulationState(nextCar)
    } catch (error) {
      console.error('Failed to restart simulation state', error)
    }
  }, [handleCancel])

  const commitDrag = useCallback(
    async (finalX: number, finalY: number, startX: number, startY: number) => {
      const state: CarState = {
        x: startX,
        y: startY,
        yaw: Math.atan2(finalY - startY, finalX - startX),
        velocity: 0,
        steer: 0,
      }

      setPressedPose(null)
      setGlobalTrajectory(null)
      globalTrajectoryRef.current = null

      if (mode === 'pose') {
        setGoal(null)
        goalRef.current = null
        await handleCancel()
        try {
          await setSimulationState(state)
        } catch (error) {
          console.error('Failed to set simulation pose', error)
        }
        return
      }

      setGoal(state)
      goalRef.current = state
      setGoalUnreachable({ visible: false, x: startX, y: startY })
      await handleBrake()
      await runGlobalPlan()
    },
    [handleBrake, handleCancel, mode, runGlobalPlan],
  )

  const handleMapPrimaryDragStart = useCallback((world: { x: number; y: number }) => {
    if (!isPointInBounds(world, mapSnapshotRef.current.boundingBox)) {
      return false
    }

    setGoalUnreachable((current) => ({ ...current, visible: false }))
    dragStartRef.current = { startX: world.x, startY: world.y }
    setPressedPose({ x: world.x, y: world.y, yaw: 0, velocity: 0, steer: 0 })
    return true
  }, [])

  const handleMapPrimaryDragMove = useCallback((world: { x: number; y: number }) => {
    const start = dragStartRef.current
    if (!start) {
      return
    }

    setPressedPose({
      x: start.startX,
      y: start.startY,
      yaw: Math.atan2(world.y - start.startY, world.x - start.startX),
      velocity: 0,
      steer: 0,
    })
  }, [])

  const handleMapPrimaryDragEnd = useCallback(
    (world: { x: number; y: number }) => {
      const currentDrag = dragStartRef.current
      if (!currentDrag) {
        return
      }

      dragStartRef.current = null
      setPressedPose(null)
      void commitDrag(world.x, world.y, currentDrag.startX, currentDrag.startY)
    },
    [commitDrag],
  )

  const handleMapPrimaryDragCancel = useCallback(() => {
    dragStartRef.current = null
    setPressedPose(null)
  }, [])

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.repeat) {
        return
      }

      switch (event.key.toLowerCase()) {
        case 'a':
          event.preventDefault()
          setMode('goal')
          break
        case 's':
          event.preventDefault()
          setMode('pose')
          break
        case 'd':
          event.preventDefault()
          void handleBrake()
          break
        case 'f':
          event.preventDefault()
          void handleCancel()
          break
        case 'r':
          event.preventDefault()
          void handleRestart()
          break
        default:
          break
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [handleBrake, handleCancel, handleRestart])

  return (
    <div className="app-shell">
      <main ref={dashboardGridRef} className={`dashboard-grid dashboard-grid--${dashboardLayout}`}>
        <section className="panel panel-map">
          <div className="panel-heading">
            <h2>Visualization</h2>
            <span className="panel-status">Timestamp: {timestamp.toFixed(1)}s</span>
          </div>

          <MapViewport
            bounds={mapSnapshot.boundingBox}
            mode={mode}
            carShape={carShape}
            motionLimits={motionLimits}
            knownObstacles={mapSnapshot.knownObstacles}
            unknownObstacles={mapSnapshot.unknownObstacles}
            car={car}
            goal={goal}
            pressedPose={pressedPose}
            goalUnreachable={goalUnreachable}
            globalTrajectory={globalTrajectory ? toTrajectoryPath(globalTrajectory) : null}
            localTrajectory={localTrajectory}
            referencePoints={referencePoints}
            globalPlannerSegments={globalPlannerSegments}
            onPrimaryDragStart={handleMapPrimaryDragStart}
            onPrimaryDragMove={handleMapPrimaryDragMove}
            onPrimaryDragEnd={handleMapPrimaryDragEnd}
            onPrimaryDragCancel={handleMapPrimaryDragCancel}
          />
        </section>

        <div className={`side-stack side-stack--${dashboardLayout}`}>
          <section className="panel chart-panel">
            <div className="panel-heading compact">
              <AutoShrinkHeading text={`Velocity: ${formatFixedWithoutNegativeZero((car?.velocity ?? 0) * 3.6, 1)}km/h`} />
            </div>
            <HistoryChart
              points={velocityHistory}
              minValue={motionLimits.minSpeedKmh}
              maxValue={motionLimits.maxSpeedKmh}
              lineColor={0x9fe870}
            />
          </section>

          <section className="panel chart-panel">
            <div className="panel-heading compact">
              <AutoShrinkHeading
                text={`Steer: ${formatFixedWithoutNegativeZero(((car?.steer ?? 0) * 180) / Math.PI, 1)}°`}
              />
            </div>
            <HistoryChart
              points={steerHistory}
              minValue={-motionLimits.maxSteerDeg}
              maxValue={motionLimits.maxSteerDeg}
              lineColor={0x57d8ff}
            />
          </section>
        </div>
      </main>

      <section className="control-ribbon">
        <div className="segmented-control">
          <button className={mode === 'goal' ? 'active' : ''} onClick={() => setMode('goal')}>
            Set Goal(A)
          </button>
          <button className={mode === 'pose' ? 'active' : ''} onClick={() => setMode('pose')}>
            Set Pose(S)
          </button>
        </div>
        <button className="ghost-button" onClick={() => void handleBrake()}>
          Brake(D)
        </button>
        <button className="ghost-button" onClick={() => void handleCancel()}>
          Cancel(F)
        </button>
        <button className="accent-button" onClick={() => void handleRestart()}>
          Restart(R)
        </button>
      </section>
    </div>
  )
}

export default App

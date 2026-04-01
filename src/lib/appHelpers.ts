import type { WasmConfigSnapshot } from './wasmCore'
import type { CarShape, MotionLimits } from './appTypes'
import type { MapBoundingBox } from './mapServerNode'
import type { HybridAStarStartSeedPoint, LocalPlannerReferencePoint, LocalPlannerTrajectoryPoint } from './wasmCore'

export function createCarShape(snapshot: WasmConfigSnapshot): CarShape {
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

export function createMotionLimits(snapshot: WasmConfigSnapshot): MotionLimits {
  return {
    scanRadius: snapshot.scanRadius,
    maxSpeedKmh: snapshot.maxSpeed * 3.6,
    minSpeedKmh: snapshot.minSpeed * 3.6,
    maxSteerDeg: (snapshot.maxSteer * 180) / Math.PI,
  }
}

export function isPointInBounds(point: { x: number; y: number }, bounds: MapBoundingBox) {
  return point.x >= bounds.minX && point.x <= bounds.maxX && point.y >= bounds.minY && point.y <= bounds.maxY
}

export function toTrajectoryPath(points: LocalPlannerTrajectoryPoint[]) {
  return points.map((point) => ({ x: point.x, y: point.y }))
}

export function toHybridAStarStartSeed(points: LocalPlannerReferencePoint[]): HybridAStarStartSeedPoint[] {
  return points.map((point) => ({
    x: point.x,
    y: point.y,
    yaw: point.yaw,
    velocity: point.velocity,
  }))
}

export function formatFixedWithoutNegativeZero(value: number, digits: number) {
  const roundedValue = Number(value.toFixed(digits))
  return (Object.is(roundedValue, -0) ? 0 : roundedValue).toFixed(digits)
}

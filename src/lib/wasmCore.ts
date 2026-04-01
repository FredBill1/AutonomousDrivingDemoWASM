import type { PathPoint } from './appModel'
import { createWorkerRpc } from './workerRpc'

export type WasmCarState = {
  x: number
  y: number
  yaw: number
  velocity: number
  steer: number
}

export type WasmConfigSnapshot = {
  wheelBase: number
  length: number
  width: number
  backToWheel: number
  wheelLength: number
  wheelWidth: number
  wheelSpacing: number
  backToCenter: number
  collisionLength: number
  collisionWidth: number
  collisionRadius: number
  targetMaxSteer: number
  maxSteer: number
  maxSteerSpeed: number
  maxSpeed: number
  minSpeed: number
  maxAccel: number
  maxCentripetalAccel: number
  targetSpeed: number
  targetMinTurningRadius: number
  scanRadius: number
}

export type HybridAStarSolution = {
  token: number
  path: PathPoint[]
  directions: number[]
  exploredSegments: Array<{ x1: number; y1: number; x2: number; y2: number }>
  exploredCount: number
  analyticExpansions: number
}

export type HybridAStarStartSeedPoint = {
  x: number
  y: number
  yaw: number
  velocity: number
}

export type HybridAStarProgress = {
  token: number
  segments: Array<{ x1: number; y1: number; x2: number; y2: number }>
  exploredCount: number
  analyticExpansions: number
}

export type LocalPlannerTrajectoryPoint = {
  x: number
  y: number
  yaw: number
  direction: number
}

export type LocalPlannerPathPoint = {
  x: number
  y: number
  yaw: number
}

export type LocalPlannerReferencePoint = {
  x: number
  y: number
  velocity: number
  yaw: number
}

export type LocalPlannerControlPoint = {
  timestamp: number
  targetVelocity: number
  targetSteer: number
}

export type LocalPlannerUpdateResult = {
  controlSequence: LocalPlannerControlPoint[]
  localTrajectory: LocalPlannerPathPoint[]
  referencePoints: LocalPlannerReferencePoint[]
  brakeTrajectory: LocalPlannerReferencePoint[]
}

export type LocalPlannerUpdateEvent = LocalPlannerUpdateResult

export type SimulationStateEvent = {
  timestamp: number
  state: WasmCarState
}

let progressListener: ((progress: HybridAStarProgress) => void) | null = null
let simulationListener: ((event: SimulationStateEvent) => void) | null = null
let localPlannerUpdateListener: ((event: LocalPlannerUpdateEvent) => void) | null = null

const computeRpc = createWorkerRpc(
  () => new Worker(new URL('./computeWorker.ts', import.meta.url), { type: 'module' }),
  (message) => {
    if (message.type === 'hybridAStarProgress' && progressListener) {
      progressListener(message.payload as HybridAStarProgress)
      return
    }

    if (message.type === 'simulationState' && simulationListener) {
      simulationListener(message.payload as SimulationStateEvent)
      return
    }

    if (message.type === 'localPlannerUpdate' && localPlannerUpdateListener) {
      localPlannerUpdateListener(message.payload as LocalPlannerUpdateEvent)
    }
  },
)

export async function ensureWasmCore() {
  await computeRpc.call('getCarConfigSnapshot')
}

export function stepCarState(
  current: WasmCarState,
  targetVelocity: number,
  targetSteer: number,
  dt: number,
) {
  return computeRpc.call<WasmCarState>('stepCarState', { current, targetVelocity, targetSteer, dt })
}

export function initSimulation(
  state: WasmCarState,
  timestamp = 0,
  simDeltaTime = 0.015,
  simulationIntervalMs = 20,
  publishIntervalMs = 50,
) {
  return computeRpc.call<null>('initSimulation', {
    state,
    timestamp,
    simDeltaTime,
    simulationIntervalMs,
    publishIntervalMs,
  })
}

export function setSimulationState(state: WasmCarState, timestamp?: number) {
  return computeRpc.call<null>('setSimulationState', { state, timestamp })
}

export function setSimulationControlSequence(
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>,
) {
  return computeRpc.call<null>('setSimulationControlSequence', { controlSequence })
}

export function stopSimulationMotion() {
  return computeRpc.call<null>('stopSimulationMotion')
}

export function resumeSimulationMotion() {
  return computeRpc.call<null>('resumeSimulationMotion')
}

export function stopSimulation() {
  return computeRpc.call<null>('stopSimulation')
}

export function checkCollision(state: WasmCarState, obstacleCoordinates: Float64Array) {
  return computeRpc.call<boolean>('checkCollision', {
    state,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  })
}

export function checkPathCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return computeRpc.call<boolean>('checkPathCollision', {
    path,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  })
}

export function checkTrajectoryCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return computeRpc.call<boolean>('checkTrajectoryCollision', {
    path,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  })
}

export function getCarConfigSnapshot() {
  return computeRpc.call<WasmConfigSnapshot>('getCarConfigSnapshot')
}

export function solveHybridAStar(
  start: WasmCarState | HybridAStarStartSeedPoint[],
  goal: WasmCarState,
  obstacleCoordinates: Float64Array,
  maxIterations: number,
  requestToken?: number,
) {
  return computeRpc.call<HybridAStarSolution | null>('solveHybridAStar', {
    start,
    startIsTrajectorySeed: Array.isArray(start),
    goal,
    obstacleCoordinates: Array.from(obstacleCoordinates),
    maxIterations: maxIterations,
    requestToken,
  })
}

export function cancelHybridAStar() {
  return computeRpc.call<null>('cancelHybridAStar')
}

export function setLocalPlannerTrajectory(trajectory: LocalPlannerTrajectoryPoint[] | null) {
  return computeRpc.call<null>('setLocalPlannerTrajectory', { trajectory })
}

export function setLocalPlannerState(
  state: WasmCarState,
  timestamp: number,
  dt?: number,
  updateIntervalMs?: number,
) {
  return computeRpc.call<null>('setLocalPlannerState', { state, timestamp, dt, updateIntervalMs })
}

export function brakeLocalPlanner() {
  return computeRpc.call<null>('brakeLocalPlanner')
}

export function cancelLocalPlanner() {
  return computeRpc.call<null>('cancelLocalPlanner')
}

export function setLocalPlannerUpdateListener(
  listener: ((event: LocalPlannerUpdateEvent) => void) | null,
) {
  localPlannerUpdateListener = listener
}

export function setHybridAStarProgressListener(
  listener: ((progress: HybridAStarProgress) => void) | null,
) {
  progressListener = listener
}

export function setSimulationStateListener(
  listener: ((event: SimulationStateEvent) => void) | null,
) {
  simulationListener = listener
}

export function resetComputeWorker(reason?: string) {
  progressListener = null
  simulationListener = null
  localPlannerUpdateListener = null
  computeRpc.reset(reason)
}

import type { PathPoint } from './appModel';
import { createWorkerRpc } from './workerRpc';
import { type LocalPlannerControlPoint, type LocalPlannerUpdateResult, type WasmCarState } from './workerTypes';

export type {
  LocalPlannerControlPoint,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerUpdateResult,
  WasmCarState,
} from './workerTypes';

export type WasmConfigSnapshot = {
  wheelBase: number;
  length: number;
  width: number;
  backToWheel: number;
  wheelLength: number;
  wheelWidth: number;
  wheelSpacing: number;
  backToCenter: number;
  collisionLength: number;
  collisionWidth: number;
  collisionRadius: number;
  targetMaxSteer: number;
  maxSteer: number;
  maxSteerSpeed: number;
  maxSpeed: number;
  minSpeed: number;
  maxAccel: number;
  maxCentripetalAccel: number;
  targetSpeed: number;
  targetMinTurningRadius: number;
  scanRadius: number;
};

export type HybridAStarSolution = {
  token: number;
  path: PathPoint[];
  directions: number[];
  exploredSegments: Array<{ x1: number; y1: number; x2: number; y2: number }>;
  exploredCount: number;
  analyticExpansions: number;
};

export type HybridAStarStartSeedPoint = {
  x: number;
  y: number;
  yaw: number;
  velocity: number;
};

export type HybridAStarProgress = {
  token: number;
  segments: Array<{ x1: number; y1: number; x2: number; y2: number }>;
  exploredCount: number;
  analyticExpansions: number;
};

export type LocalPlannerTrajectoryPoint = {
  x: number;
  y: number;
  yaw: number;
  direction: number;
};

export type LocalPlannerUpdateEvent = LocalPlannerUpdateResult;

export type SimulationStateEvent = {
  timestamp: number;
  state: WasmCarState;
};

let progressListener: ((progress: HybridAStarProgress) => void) | null = null;
let simulationListener: ((event: SimulationStateEvent) => void) | null = null;
let localPlannerUpdateListener: ((event: LocalPlannerUpdateEvent) => void) | null = null;

const computeRpc = createWorkerRpc(
  () => new Worker(new URL('./computeWorker.ts', import.meta.url), { type: 'module' }),
  (message) => {
    if (message.type === 'hybridAStarProgress' && progressListener) {
      progressListener(message.payload as HybridAStarProgress);
      return;
    }

    if (message.type === 'simulationState' && simulationListener) {
      simulationListener(message.payload as SimulationStateEvent);
      return;
    }

    if (message.type === 'localPlannerUpdate' && localPlannerUpdateListener) {
      localPlannerUpdateListener(message.payload as LocalPlannerUpdateEvent);
    }
  },
);

function callCompute<TResult>(type: string, payload?: unknown) {
  return computeRpc.call<TResult>(type, payload);
}

function serializeObstacleCoordinates(obstacleCoordinates: Float64Array) {
  return Array.from(obstacleCoordinates);
}

export async function ensureWasmCore() {
  await callCompute('getCarConfigSnapshot');
}

export function stepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  return callCompute<WasmCarState>('stepCarState', { current, targetVelocity, targetSteer, dt });
}

export function initSimulation(
  state: WasmCarState,
  timestamp = 0,
  simDeltaTime = 0.015,
  simulationIntervalMs = 20,
  publishIntervalMs = 50,
) {
  return callCompute<null>('initSimulation', {
    state,
    timestamp,
    simDeltaTime,
    simulationIntervalMs,
    publishIntervalMs,
  });
}

export function setSimulationState(state: WasmCarState, timestamp?: number) {
  return callCompute<null>('setSimulationState', { state, timestamp });
}

export function setSimulationControlSequence(controlSequence: LocalPlannerControlPoint[]) {
  return callCompute<null>('setSimulationControlSequence', { controlSequence });
}

export function stopSimulationMotion() {
  return callCompute<null>('stopSimulationMotion');
}

export function resumeSimulationMotion() {
  return callCompute<null>('resumeSimulationMotion');
}

export function stopSimulation() {
  return callCompute<null>('stopSimulation');
}

export function checkCollision(state: WasmCarState, obstacleCoordinates: Float64Array) {
  return callCompute<boolean>('checkCollision', {
    state,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function checkPathCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return callCompute<boolean>('checkPathCollision', {
    path,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function checkTrajectoryCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return callCompute<boolean>('checkTrajectoryCollision', {
    path,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function getCarConfigSnapshot() {
  return callCompute<WasmConfigSnapshot>('getCarConfigSnapshot');
}

export function solveHybridAStar(
  start: WasmCarState | HybridAStarStartSeedPoint[],
  goal: WasmCarState,
  obstacleCoordinates: Float64Array,
  maxIterations: number,
  requestToken?: number,
) {
  return callCompute<HybridAStarSolution | null>('solveHybridAStar', {
    start,
    startIsTrajectorySeed: Array.isArray(start),
    goal,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
    maxIterations,
    requestToken,
  });
}

export function cancelHybridAStar() {
  return callCompute<null>('cancelHybridAStar');
}

export function setLocalPlannerTrajectory(trajectory: LocalPlannerTrajectoryPoint[] | null) {
  return callCompute<null>('setLocalPlannerTrajectory', { trajectory });
}

export function setLocalPlannerState(state: WasmCarState, timestamp: number, updateIntervalMs?: number) {
  return callCompute<null>('setLocalPlannerState', { state, timestamp, updateIntervalMs });
}

export function brakeLocalPlanner() {
  return callCompute<null>('brakeLocalPlanner');
}

export function cancelLocalPlanner() {
  return callCompute<null>('cancelLocalPlanner');
}

export function setLocalPlannerUpdateListener(listener: ((event: LocalPlannerUpdateEvent) => void) | null) {
  localPlannerUpdateListener = listener;
}

export function setHybridAStarProgressListener(listener: ((progress: HybridAStarProgress) => void) | null) {
  progressListener = listener;
}

export function setSimulationStateListener(listener: ((event: SimulationStateEvent) => void) | null) {
  simulationListener = listener;
}

export function resetComputeWorker(reason?: string) {
  progressListener = null;
  simulationListener = null;
  localPlannerUpdateListener = null;
  computeRpc.reset(reason);
}

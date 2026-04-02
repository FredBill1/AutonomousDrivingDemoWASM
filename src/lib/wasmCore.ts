import { createWorkerRpc } from './workerRpc';
import type {
  HybridAStarProgress,
  HybridAStarStartSeedPoint,
  LocalPlannerControlPoint,
  LocalPlannerTrajectoryPoint,
  LocalPlannerUpdateResult,
  SimulationStateEvent,
  WasmCarState,
  WorkerEventMap,
  WorkerMethodMap,
} from './workerTypes';

export type {
  HybridAStarProgress,
  HybridAStarProgressSegment,
  HybridAStarSolution,
  HybridAStarStartSeedPoint,
  LocalPlannerControlPoint,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerTrajectoryPoint,
  LocalPlannerUpdateResult,
  SimulationStateEvent,
  WasmCarState,
  WasmConfigSnapshot,
} from './workerTypes';

const computeRpc = createWorkerRpc<WorkerMethodMap, WorkerEventMap>(
  () => new Worker(new URL('./computeWorker.ts', import.meta.url), { type: 'module' }),
  (event) => {
    switch (event.type) {
      case 'hybridAStarProgress':
        eventListeners.hybridAStarProgress?.(event.payload);
        break;
      case 'simulationState':
        eventListeners.simulationState?.(event.payload);
        break;
      case 'localPlannerUpdate':
        eventListeners.localPlannerUpdate?.(event.payload);
        break;
      default:
        break;
    }
  },
);

const eventListeners: {
  [Key in keyof WorkerEventMap]: ((payload: WorkerEventMap[Key]) => void) | null;
} = {
  hybridAStarProgress: null,
  simulationState: null,
  localPlannerUpdate: null,
};

function serializeObstacleCoordinates(obstacleCoordinates: Float64Array) {
  return Array.from(obstacleCoordinates);
}

function setEventListener<Key extends keyof WorkerEventMap>(
  type: Key,
  listener: ((payload: WorkerEventMap[Key]) => void) | null,
) {
  switch (type) {
    case 'hybridAStarProgress':
      eventListeners.hybridAStarProgress = listener as ((payload: HybridAStarProgress) => void) | null;
      break;
    case 'simulationState':
      eventListeners.simulationState = listener as ((payload: SimulationStateEvent) => void) | null;
      break;
    case 'localPlannerUpdate':
      eventListeners.localPlannerUpdate = listener as ((payload: LocalPlannerUpdateResult) => void) | null;
      break;
    default:
      break;
  }
}

export async function ensureWasmCore() {
  await computeRpc.call('getCarConfigSnapshot');
}

export function stepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  return computeRpc.call('stepCarState', { current, targetVelocity, targetSteer, dt });
}

export function initSimulation(
  state: WasmCarState,
  timestamp = 0,
  simDeltaTime = 0.015,
  simulationIntervalMs = 20,
  publishIntervalMs = 50,
) {
  return computeRpc.call('initSimulation', {
    state,
    timestamp,
    simDeltaTime,
    simulationIntervalMs,
    publishIntervalMs,
  });
}

export function setSimulationState(state: WasmCarState, timestamp?: number) {
  return computeRpc.call('setSimulationState', { state, timestamp });
}

export function setSimulationControlSequence(controlSequence: LocalPlannerControlPoint[]) {
  return computeRpc.call('setSimulationControlSequence', { controlSequence });
}

export function stopSimulationMotion() {
  return computeRpc.call('stopSimulationMotion');
}

export function resumeSimulationMotion() {
  return computeRpc.call('resumeSimulationMotion');
}

export function stopSimulation() {
  return computeRpc.call('stopSimulation');
}

export function checkCollision(state: WasmCarState, obstacleCoordinates: Float64Array) {
  return computeRpc.call('checkCollision', {
    state,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function checkPathCollision(
  path: Array<{ x: number; y: number; yaw: number }>,
  obstacleCoordinates: Float64Array,
) {
  return computeRpc.call('checkPathCollision', {
    path,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function checkTrajectoryCollision(
  path: Array<{ x: number; y: number; yaw: number }>,
  obstacleCoordinates: Float64Array,
) {
  return computeRpc.call('checkTrajectoryCollision', {
    path,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
  });
}

export function getCarConfigSnapshot() {
  return computeRpc.call('getCarConfigSnapshot');
}

export function solveHybridAStar(
  start: WasmCarState | HybridAStarStartSeedPoint[],
  goal: WasmCarState,
  obstacleCoordinates: Float64Array,
  maxIterations: number,
  requestToken?: number,
) {
  return computeRpc.call('solveHybridAStar', {
    start,
    startIsTrajectorySeed: Array.isArray(start),
    goal,
    obstacleCoordinates: serializeObstacleCoordinates(obstacleCoordinates),
    maxIterations,
    requestToken,
  });
}

export function cancelHybridAStar() {
  return computeRpc.call('cancelHybridAStar');
}

export function setLocalPlannerTrajectory(trajectory: LocalPlannerTrajectoryPoint[] | null) {
  return computeRpc.call('setLocalPlannerTrajectory', { trajectory });
}

export function setLocalPlannerState(state: WasmCarState, timestamp: number, updateIntervalMs?: number) {
  return computeRpc.call('setLocalPlannerState', { state, timestamp, updateIntervalMs });
}

export function brakeLocalPlanner() {
  return computeRpc.call('brakeLocalPlanner');
}

export function cancelLocalPlanner() {
  return computeRpc.call('cancelLocalPlanner');
}

export function setLocalPlannerUpdateListener(listener: ((event: LocalPlannerUpdateResult) => void) | null) {
  setEventListener('localPlannerUpdate', listener);
}

export function setHybridAStarProgressListener(listener: ((progress: HybridAStarProgress) => void) | null) {
  setEventListener('hybridAStarProgress', listener);
}

export function setSimulationStateListener(listener: ((event: SimulationStateEvent) => void) | null) {
  setEventListener('simulationState', listener);
}

export function resetComputeWorker(reason?: string) {
  setEventListener('hybridAStarProgress', null);
  setEventListener('simulationState', null);
  setEventListener('localPlannerUpdate', null);
  computeRpc.reset(reason);
}

import type { PathPoint } from './appModel';
import { createWorkerRpc } from './workerRpc';
import { type LocalPlannerControlPoint, type LocalPlannerUpdateResult, type WasmCarState } from './workerTypes';
import type { CarShape } from './appTypes';

export type {
  LocalPlannerControlPoint,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerUpdateResult,
  WasmCarState,
} from './workerTypes';

export type WasmConfigSnapshot = CarShape & {
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

type HybridAStarSegment = { x1: number; y1: number; x2: number; y2: number };

type HybridAStarMetrics = {
  token: number;
  exploredCount: number;
  analyticExpansions: number;
};

export type HybridAStarSolution = HybridAStarMetrics & {
  path: PathPoint[];
  directions: number[];
  exploredSegments: HybridAStarSegment[];
};

export type HybridAStarStartSeedPoint = {
  x: number;
  y: number;
  yaw: number;
  velocity: number;
};

export type HybridAStarProgress = HybridAStarMetrics & {
  segments: HybridAStarSegment[];
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

type WorkerMethod = Parameters<typeof computeRpc.call>[0];

function callWorker<T>(method: WorkerMethod): Promise<T>;
function callWorker<T, P>(method: WorkerMethod, payload: P): Promise<T>;
function callWorker<T, P>(method: WorkerMethod, payload?: P) {
  return payload === undefined
    ? computeRpc.call<T>(method)
    : computeRpc.call<T>(method, payload as never);
}

const callWorkerVoid = (method: WorkerMethod) => callWorker<null>(method);
const callVoidMethod = (method: WorkerMethod) => () => callWorkerVoid(method);
const voidCalls = {
  stopSimulationMotion: callVoidMethod('stopSimulationMotion'),
  resumeSimulationMotion: callVoidMethod('resumeSimulationMotion'),
  stopSimulation: callVoidMethod('stopSimulation'),
  cancelHybridAStar: callVoidMethod('cancelHybridAStar'),
  brakeLocalPlanner: callVoidMethod('brakeLocalPlanner'),
  cancelLocalPlanner: callVoidMethod('cancelLocalPlanner'),
};

export async function ensureWasmCore() {
  await callWorker('getCarConfigSnapshot');
}

export function stepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  return callWorker<WasmCarState>('stepCarState', { current, targetVelocity, targetSteer, dt });
}

export function initSimulation(
  state: WasmCarState,
  timestamp = 0,
  simDeltaTime = 0.015,
  simulationIntervalMs = 20,
  publishIntervalMs = 50,
) {
  return callWorker<null>('initSimulation', {
    state,
    timestamp,
    simDeltaTime,
    simulationIntervalMs,
    publishIntervalMs,
  });
}

export function setSimulationState(state: WasmCarState, timestamp?: number) {
  return callWorker<null>('setSimulationState', { state, timestamp });
}

export function setSimulationControlSequence(controlSequence: LocalPlannerControlPoint[]) {
  return callWorker<null>('setSimulationControlSequence', { controlSequence });
}

export const {
  stopSimulationMotion,
  resumeSimulationMotion,
  stopSimulation,
  cancelHybridAStar,
  brakeLocalPlanner,
  cancelLocalPlanner,
} = voidCalls;

export function checkCollision(state: WasmCarState, obstacleCoordinates: Float64Array) {
  return callWorker<boolean>('checkCollision', {
    state,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  });
}

export function checkPathCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return callWorker<boolean>('checkPathCollision', {
    path,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  });
}

export function checkTrajectoryCollision(path: PathPoint[], obstacleCoordinates: Float64Array) {
  return callWorker<boolean>('checkTrajectoryCollision', {
    path,
    obstacleCoordinates: Array.from(obstacleCoordinates),
  });
}

export function getCarConfigSnapshot() {
  return callWorker<WasmConfigSnapshot>('getCarConfigSnapshot');
}

export function solveHybridAStar(
  start: WasmCarState | HybridAStarStartSeedPoint[],
  goal: WasmCarState,
  obstacleCoordinates: Float64Array,
  maxIterations: number,
  requestToken?: number,
) {
  return callWorker<HybridAStarSolution | null>('solveHybridAStar', {
    start,
    startIsTrajectorySeed: Array.isArray(start),
    goal,
    obstacleCoordinates: Array.from(obstacleCoordinates),
    maxIterations: maxIterations,
    requestToken,
  });
}

export function setLocalPlannerTrajectory(trajectory: LocalPlannerTrajectoryPoint[] | null) {
  return callWorker<null>('setLocalPlannerTrajectory', { trajectory });
}

export function setLocalPlannerState(state: WasmCarState, timestamp: number, updateIntervalMs?: number) {
  return callWorker<null>('setLocalPlannerState', { state, timestamp, updateIntervalMs });
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

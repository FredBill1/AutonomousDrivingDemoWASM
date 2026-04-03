import { getDefaultControllerConfig } from './controllerConfig';
import type {
  HybridAStarProgress,
  HybridAStarStartSeedPoint,
  LocalPlannerTrajectoryPoint,
  LocalPlannerUpdateResult,
  OrchestratorEventMap,
  OrchestratorMethodMap,
  SimulationStateEvent,
  WasmCarState,
} from './workerContracts';
import { createPubSub } from './workerEventBus';
import { createWorkerRpc } from './workerRpc';

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
} from './workerContracts';

const controllerConfig = getDefaultControllerConfig();
const eventBus = createPubSub<OrchestratorEventMap>();
const orchestratorRpc = createWorkerRpc<OrchestratorMethodMap, OrchestratorEventMap>(
  () => new Worker(new URL('./orchestrationWorker.ts', import.meta.url), { type: 'module' }),
  (event) => {
    eventBus.publish(event.type, event.payload);
  },
);

let initializationPromise: Promise<void> | null = null;

function subscribeToEvent<Key extends keyof OrchestratorEventMap>(
  type: Key,
  listener: ((payload: OrchestratorEventMap[Key]) => void) | null,
) {
  if (!listener) {
    return () => undefined;
  }
  return eventBus.subscribe(type, listener);
}

export async function ensureWasmCore() {
  if (!initializationPromise) {
    initializationPromise = orchestratorRpc.call('initializeRuntime', controllerConfig).then(() => undefined);
  }
  await initializationPromise;
}

export async function getCarConfigSnapshot() {
  await ensureWasmCore();
  return orchestratorRpc.call('getCarConfigSnapshot');
}

export async function stepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  await ensureWasmCore();
  return orchestratorRpc.call('stepCarState', { current, targetVelocity, targetSteer, dt });
}

export async function initSimulation(state: WasmCarState, timestamp = 0) {
  await ensureWasmCore();
  return orchestratorRpc.call('initSimulation', { state, timestamp });
}

export async function setSimulationState(state: WasmCarState, timestamp?: number) {
  await ensureWasmCore();
  return orchestratorRpc.call('setSimulationState', { state, timestamp });
}

export async function stopSimulationMotion() {
  await ensureWasmCore();
  return orchestratorRpc.call('stopSimulationMotion');
}

export async function resumeSimulationMotion() {
  await ensureWasmCore();
  return orchestratorRpc.call('resumeSimulationMotion');
}

export async function stopSimulation() {
  await ensureWasmCore();
  return orchestratorRpc.call('stopSimulation');
}

export async function checkCollision(state: WasmCarState, obstacleCoordinates: Float64Array) {
  await ensureWasmCore();
  return orchestratorRpc.call('checkCollision', { state, obstacleCoordinates });
}

export async function checkPathCollision(
  path: Array<{ x: number; y: number; yaw: number }>,
  obstacleCoordinates: Float64Array,
) {
  await ensureWasmCore();
  return orchestratorRpc.call('checkPathCollision', { path, obstacleCoordinates });
}

export async function checkTrajectoryCollision(
  path: Array<{ x: number; y: number; yaw: number }>,
  obstacleCoordinates: Float64Array,
) {
  await ensureWasmCore();
  return orchestratorRpc.call('checkTrajectoryCollision', { path, obstacleCoordinates });
}

export async function solveHybridAStar(
  start: WasmCarState | HybridAStarStartSeedPoint[],
  goal: WasmCarState,
  obstacleCoordinates: Float64Array,
  maxIterations: number,
  requestToken?: number,
) {
  await ensureWasmCore();
  return orchestratorRpc.call('solveHybridAStar', {
    start,
    startIsTrajectorySeed: Array.isArray(start),
    goal,
    obstacleCoordinates,
    maxIterations,
    requestToken,
  });
}

export async function cancelHybridAStar() {
  await ensureWasmCore();
  return orchestratorRpc.call('cancelHybridAStar');
}

export async function setLocalPlannerTrajectory(trajectory: LocalPlannerTrajectoryPoint[] | null) {
  await ensureWasmCore();
  return orchestratorRpc.call('setLocalPlannerTrajectory', { trajectory });
}

export async function brakeLocalPlanner() {
  await ensureWasmCore();
  return orchestratorRpc.call('brakeLocalPlanner');
}

export async function cancelLocalPlanner() {
  await ensureWasmCore();
  return orchestratorRpc.call('cancelLocalPlanner');
}

let clearLocalPlannerUpdateSubscription: () => void = () => undefined;
let clearHybridAStarProgressSubscription: () => void = () => undefined;
let clearSimulationStateSubscription: () => void = () => undefined;

export function setLocalPlannerUpdateListener(listener: ((event: LocalPlannerUpdateResult) => void) | null) {
  clearLocalPlannerUpdateSubscription();
  clearLocalPlannerUpdateSubscription = subscribeToEvent('localPlannerUpdate', listener);
}

export function setHybridAStarProgressListener(listener: ((progress: HybridAStarProgress) => void) | null) {
  clearHybridAStarProgressSubscription();
  clearHybridAStarProgressSubscription = subscribeToEvent('hybridAStarProgress', listener);
}

export function setSimulationStateListener(listener: ((event: SimulationStateEvent) => void) | null) {
  clearSimulationStateSubscription();
  clearSimulationStateSubscription = subscribeToEvent('simulationState', listener);
}

export function resetComputeWorker(reason?: string) {
  clearHybridAStarProgressSubscription();
  clearSimulationStateSubscription();
  clearLocalPlannerUpdateSubscription();
  initializationPromise = null;
  eventBus.clear();
  orchestratorRpc.reset(reason);
}

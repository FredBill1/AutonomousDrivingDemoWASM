import initWasm, { CarState, path_check_collision, trajectory_check_collision } from '../../wasm-core/pkg/wasm_core';

import { encodeFlatTuplesToFloat64 } from './flatCodec';
import { createPubSub } from './workerEventBus';
import { createWorkerRpc } from './workerRpc';
import type {
  GlobalPlannerWorkerEventMap,
  GlobalPlannerWorkerMethodMap,
  LocalPlannerWorkerEventMap,
  LocalPlannerWorkerMethodMap,
  OrchestratorEventMap,
  OrchestratorMethodMap,
  SimulationWorkerEventMap,
  SimulationWorkerMethodMap,
  WasmPose,
  WorkerEvent,
  WorkerHandlerMap,
  WorkerRequest,
  WorkerResponse,
  WasmRuntime,
} from './workerContracts';
import { createCarConfig, toWasmConfigSnapshot } from './wasmConfig';

const domainEvents = createPubSub<OrchestratorEventMap>();
const simulationRpc = createWorkerRpc<SimulationWorkerMethodMap, SimulationWorkerEventMap>(
  () => new Worker(new URL('./simulationWorker.ts', import.meta.url), { type: 'module' }),
  (event) => {
    domainEvents.publish(event.type, event.payload);
  },
);
const localPlannerRpc = createWorkerRpc<LocalPlannerWorkerMethodMap, LocalPlannerWorkerEventMap>(
  () => new Worker(new URL('./localPlannerWorker.ts', import.meta.url), { type: 'module' }),
  (event) => {
    domainEvents.publish(event.type, event.payload);
  },
);
const globalPlannerRpc = createWorkerRpc<GlobalPlannerWorkerMethodMap, GlobalPlannerWorkerEventMap>(
  () => new Worker(new URL('./globalPlannerWorker.ts', import.meta.url), { type: 'module' }),
  (event) => {
    domainEvents.publish(event.type, event.payload);
  },
);

const workerState: {
  controllerConfig: OrchestratorMethodMap['initializeRuntime']['payload'] | null;
  collisionRuntime: WasmRuntime | null;
  subscriptionsRegistered: boolean;
} = {
  controllerConfig: null,
  collisionRuntime: null,
  subscriptionsRegistered: false,
};

function postEvent<Key extends keyof OrchestratorEventMap>(type: Key, payload: OrchestratorEventMap[Key]) {
  self.postMessage({ type, payload } satisfies WorkerEvent<OrchestratorEventMap>);
}

function registerSubscriptions() {
  if (workerState.subscriptionsRegistered) {
    return;
  }
  workerState.subscriptionsRegistered = true;

  domainEvents.subscribe('simulationState', (payload) => {
    postEvent('simulationState', payload);
    void localPlannerRpc.call('setLocalPlannerState', {
      state: payload.state,
      timestamp: payload.timestamp,
    }).catch((error) => {
      console.error('Failed to forward simulation state to local planner', error);
    });
  });

  domainEvents.subscribe('localPlannerUpdate', (payload) => {
    postEvent('localPlannerUpdate', payload);
    void simulationRpc.call('setSimulationControlSequence', {
      controlSequence: payload.controlSequence,
    }).catch((error) => {
      console.error('Failed to forward local planner controls to simulation', error);
    });
  });

  domainEvents.subscribe('hybridAStarProgress', (payload) => {
    postEvent('hybridAStarProgress', payload);
  });
}

async function ensureCollisionRuntime() {
  if (!workerState.controllerConfig) {
    throw new Error('Controller runtime not initialized');
  }
  if (!workerState.collisionRuntime) {
    await initWasm();
    workerState.collisionRuntime = {
      carConfig: createCarConfig(workerState.controllerConfig.carConfig),
      hybridAStarConfig: null as never,
      mpcConfig: null as never,
    };
  }
  return workerState.collisionRuntime;
}

function encodePath(path: WasmPose[]) {
  return encodeFlatTuplesToFloat64(path, (point) => [point.x, point.y, point.yaw]);
}

const handlers: WorkerHandlerMap<OrchestratorMethodMap> = {
  async initializeRuntime(payload) {
    workerState.controllerConfig = payload;
    workerState.collisionRuntime = null;
    registerSubscriptions();
    await Promise.all([
      simulationRpc.call('initializeRuntime', { carConfig: payload.carConfig, runtime: payload.runtime }),
      localPlannerRpc.call('initializeRuntime', {
        carConfig: payload.carConfig,
        mpcConfig: payload.mpcConfig,
        runtime: payload.runtime,
      }),
      globalPlannerRpc.call('initializeRuntime', {
        carConfig: payload.carConfig,
        hybridAStarConfig: payload.hybridAStarConfig,
        runtime: payload.runtime,
      }),
    ]);
    return null;
  },

  async getCarConfigSnapshot() {
    if (!workerState.controllerConfig) {
      throw new Error('Controller runtime not initialized');
    }
    return toWasmConfigSnapshot(workerState.controllerConfig.carConfig);
  },

  stepCarState(payload) {
    return simulationRpc.call('stepCarState', payload);
  },

  initSimulation(payload) {
    return simulationRpc.call('initSimulation', payload);
  },

  setSimulationState(payload) {
    return simulationRpc.call('setSimulationState', payload);
  },

  stopSimulationMotion() {
    return simulationRpc.call('stopSimulationMotion');
  },

  resumeSimulationMotion() {
    return simulationRpc.call('resumeSimulationMotion');
  },

  async stopSimulation() {
    await Promise.all([
      simulationRpc.call('stopSimulation'),
      localPlannerRpc.call('stopLocalPlanner'),
      globalPlannerRpc.call('cancelHybridAStar').catch(() => null),
    ]);
    return null;
  },

  async checkCollision(payload) {
    const { carConfig } = await ensureCollisionRuntime();
    const state = new CarState(
      payload.state.x,
      payload.state.y,
      payload.state.yaw,
      payload.state.velocity,
      payload.state.steer,
    );
    try {
      return state.check_collision(carConfig, payload.obstacleCoordinates);
    } finally {
      state.free();
    }
  },

  async checkPathCollision(payload) {
    const { carConfig } = await ensureCollisionRuntime();
    return path_check_collision(carConfig, encodePath(payload.path), payload.obstacleCoordinates);
  },

  async checkTrajectoryCollision(payload) {
    const { carConfig } = await ensureCollisionRuntime();
    return trajectory_check_collision(carConfig, encodePath(payload.path), payload.obstacleCoordinates);
  },

  solveHybridAStar(payload) {
    return globalPlannerRpc.call('solveHybridAStar', payload);
  },

  cancelHybridAStar() {
    return globalPlannerRpc.call('cancelHybridAStar');
  },

  setLocalPlannerTrajectory(payload) {
    return localPlannerRpc.call('setLocalPlannerTrajectory', payload);
  },

  brakeLocalPlanner() {
    return localPlannerRpc.call('brakeLocalPlanner');
  },

  cancelLocalPlanner() {
    return localPlannerRpc.call('cancelLocalPlanner');
  },

  solveReedsSheppCandidates(payload) {
    return globalPlannerRpc.call('solveReedsSheppCandidates', payload);
  },
};

function isKnownRequestType(type: string): type is keyof OrchestratorMethodMap {
  return type in handlers;
}

self.onmessage = (event: MessageEvent<WorkerRequest<OrchestratorMethodMap> | { id?: number; type?: string; payload?: unknown }>) => {
  const message = event.data;
  if (typeof message.id !== 'number' || typeof message.type !== 'string' || !isKnownRequestType(message.type)) {
    self.postMessage({
      id: typeof message.id === 'number' ? message.id : -1,
      ok: false,
      error: `Unknown worker request: ${String(message.type)}`,
    } satisfies WorkerResponse<OrchestratorMethodMap>);
    return;
  }

  void handlers[message.type](message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse<OrchestratorMethodMap>);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse<OrchestratorMethodMap>);
    });
};

export {};

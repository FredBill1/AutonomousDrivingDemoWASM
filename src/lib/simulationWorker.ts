import initWasm, { CarState } from '../../wasm-core/pkg/wasm_core';

import { createCarConfig } from './wasmConfig';
import type {
  LocalPlannerControlPoint,
  SimulationSession,
  SimulationStateEvent,
  SimulationWorkerEventMap,
  SimulationWorkerMethodMap,
  WasmCarState,
  WasmRuntime,
  WorkerEvent,
  WorkerHandlerMap,
  WorkerRequest,
  WorkerResponse,
} from './workerContracts';

const workerState: {
  runtime: WasmRuntime | null;
  session: SimulationSession | null;
  simulationDeltaTime: number;
  simulationIntervalMs: number;
  publishIntervalMs: number;
} = {
  runtime: null,
  session: null,
  simulationDeltaTime: 0.015,
  simulationIntervalMs: 20,
  publishIntervalMs: 50,
};

function ensureRuntime() {
  if (!workerState.runtime) {
    throw new Error('Simulation worker runtime not initialized');
  }
  return workerState.runtime;
}

function postEvent<Key extends keyof SimulationWorkerEventMap>(type: Key, payload: SimulationWorkerEventMap[Key]) {
  self.postMessage({ type, payload } satisfies WorkerEvent<SimulationWorkerEventMap>);
}

function cloneState(state: WasmCarState): WasmCarState {
  return { ...state };
}

function createSimulationSession(state: WasmCarState, timestamp = 0): SimulationSession {
  return {
    state: cloneState(state),
    timestamp,
    controlSequence: null,
    stopped: true,
    simulationTimerId: null,
    publishTimerId: null,
    loopToken: 0,
    stateVersion: 0,
  };
}

function requireSession() {
  if (!workerState.session) {
    throw new Error('Simulation not initialized');
  }
  return workerState.session;
}

function emitSimulationState() {
  if (!workerState.session) {
    return;
  }
  const payload: SimulationStateEvent = {
    timestamp: workerState.session.timestamp,
    state: cloneState(workerState.session.state),
  };
  postEvent('simulationState', payload);
}

function clearTimers() {
  if (!workerState.session) {
    return;
  }
  if (workerState.session.simulationTimerId !== null) {
    clearTimeout(workerState.session.simulationTimerId);
    workerState.session.simulationTimerId = null;
  }
  if (workerState.session.publishTimerId !== null) {
    clearInterval(workerState.session.publishTimerId);
    workerState.session.publishTimerId = null;
  }
}

function cloneControlSequence(controlSequence: LocalPlannerControlPoint[]): LocalPlannerControlPoint[] {
  return controlSequence.map((point) => ({ ...point }));
}

function assertValidControlSequence(controlSequence: LocalPlannerControlPoint[]) {
  if (controlSequence.length < 2) {
    throw new Error('Control sequence must contain at least two control points');
  }
  for (let index = 0; index < controlSequence.length; index += 1) {
    const point = controlSequence[index];
    if (
      !Number.isFinite(point.timestamp) ||
      !Number.isFinite(point.targetVelocity) ||
      !Number.isFinite(point.targetSteer)
    ) {
      throw new Error('Control sequence contains non-finite values');
    }
    if (index > 0 && !(controlSequence[index - 1].timestamp < point.timestamp)) {
      throw new Error('Control sequence timestamps must be strictly increasing');
    }
  }
}

function sampleControl(controlSequence: LocalPlannerControlPoint[] | null, timestamp: number) {
  if (!controlSequence || controlSequence.length === 0) {
    return null;
  }
  if (timestamp <= controlSequence[0].timestamp) {
    return controlSequence[0];
  }
  const last = controlSequence[controlSequence.length - 1];
  if (timestamp >= last.timestamp) {
    return last;
  }
  for (let index = 1; index < controlSequence.length; index += 1) {
    const next = controlSequence[index];
    if (timestamp > next.timestamp) {
      continue;
    }
    const previous = controlSequence[index - 1];
    const ratio = (timestamp - previous.timestamp) / (next.timestamp - previous.timestamp);
    return {
      timestamp,
      targetVelocity: previous.targetVelocity + (next.targetVelocity - previous.targetVelocity) * ratio,
      targetSteer: previous.targetSteer + (next.targetSteer - previous.targetSteer) * ratio,
    };
  }
  return last;
}

function computeStepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  const { carConfig } = ensureRuntime();
  const state = new CarState(current.x, current.y, current.yaw, current.velocity, current.steer);
  try {
    const next = state.stepped(carConfig, targetVelocity, targetSteer, dt);
    try {
      return {
        x: next.x,
        y: next.y,
        yaw: next.yaw,
        velocity: next.velocity,
        steer: next.steer,
      };
    } finally {
      next.free();
    }
  } finally {
    state.free();
  }
}

function advanceSimulation(session: SimulationSession) {
  const control = sampleControl(session.controlSequence, session.timestamp);
  if (!control) {
    return Promise.resolve(
      computeStepCarState(session.state, session.state.velocity, session.state.steer, workerState.simulationDeltaTime),
    );
  }
  return Promise.resolve(
    computeStepCarState(session.state, control.targetVelocity, control.targetSteer, workerState.simulationDeltaTime),
  );
}

function scheduleSimulationTick(session: SimulationSession, loopToken: number) {
  session.simulationTimerId = setTimeout(() => {
    if (!workerState.session || workerState.session !== session || session.loopToken !== loopToken) {
      return;
    }

    const stateVersion = session.stateVersion;
    session.timestamp += workerState.simulationDeltaTime;
    void advanceSimulation(session)
      .then((nextState) => {
        if (
          !workerState.session ||
          workerState.session !== session ||
          session.loopToken !== loopToken ||
          session.stateVersion !== stateVersion
        ) {
          return;
        }
        session.state = nextState;
      })
      .catch((error) => {
        console.error('Failed to advance simulation', error);
      })
      .finally(() => {
        if (!workerState.session || workerState.session !== session || session.loopToken !== loopToken) {
          return;
        }
        scheduleSimulationTick(session, loopToken);
      });
  }, workerState.simulationIntervalMs);
}

function startSimulationLoop() {
  if (!workerState.session) {
    return;
  }
  clearTimers();
  workerState.session.loopToken += 1;
  const loopToken = workerState.session.loopToken;
  scheduleSimulationTick(workerState.session, loopToken);
  workerState.session.publishTimerId = setInterval(() => {
    emitSimulationState();
  }, workerState.publishIntervalMs);
}

const handlers: WorkerHandlerMap<SimulationWorkerMethodMap> = {
  async initializeRuntime(payload) {
    await initWasm();
    workerState.runtime = {
      carConfig: createCarConfig(payload.carConfig),
      hybridAStarConfig: null as never,
      mpcConfig: null as never,
    };
    workerState.simulationDeltaTime = payload.runtime.simulationDeltaTime;
    workerState.simulationIntervalMs = payload.runtime.simulationIntervalMs;
    workerState.publishIntervalMs = payload.runtime.simulationPublishIntervalMs;
    return null;
  },

  stepCarState(payload) {
    return Promise.resolve(
      computeStepCarState(payload.current, payload.targetVelocity, payload.targetSteer, payload.dt),
    );
  },

  initSimulation(payload) {
    ensureRuntime();
    workerState.session = createSimulationSession(payload.state, payload.timestamp ?? 0);
    startSimulationLoop();
    return Promise.resolve(null);
  },

  setSimulationState(payload) {
    ensureRuntime();
    if (!workerState.session) {
      workerState.session = createSimulationSession(payload.state, payload.timestamp ?? 0);
      startSimulationLoop();
      return Promise.resolve(null);
    }
    workerState.session.state = cloneState(payload.state);
    workerState.session.controlSequence = null;
    workerState.session.stopped = true;
    workerState.session.stateVersion += 1;
    if (payload.timestamp !== undefined) {
      workerState.session.timestamp = payload.timestamp;
    }
    return Promise.resolve(null);
  },

  setSimulationControlSequence(payload) {
    const session = requireSession();
    if (session.stopped) {
      return Promise.resolve(null);
    }
    assertValidControlSequence(payload.controlSequence);
    session.controlSequence = cloneControlSequence(payload.controlSequence);
    session.stateVersion += 1;
    return Promise.resolve(null);
  },

  stopSimulationMotion() {
    const session = requireSession();
    session.state = { ...session.state, velocity: 0, steer: 0 };
    session.controlSequence = null;
    session.stopped = true;
    session.stateVersion += 1;
    return Promise.resolve(null);
  },

  resumeSimulationMotion() {
    requireSession().stopped = false;
    return Promise.resolve(null);
  },

  stopSimulation() {
    clearTimers();
    workerState.session = null;
    return Promise.resolve(null);
  },
};

function isKnownRequestType(type: string): type is keyof SimulationWorkerMethodMap {
  return type in handlers;
}

self.onmessage = (
  event: MessageEvent<WorkerRequest<SimulationWorkerMethodMap> | { id?: number; type?: string; payload?: unknown }>,
) => {
  const message = event.data;
  if (typeof message.id !== 'number' || typeof message.type !== 'string' || !isKnownRequestType(message.type)) {
    self.postMessage({
      id: typeof message.id === 'number' ? message.id : -1,
      ok: false,
      error: `Unknown worker request: ${String(message.type)}`,
    } satisfies WorkerResponse<SimulationWorkerMethodMap>);
    return;
  }

  const requestId = message.id;

  void handlers[message.type](message.payload as never)
    .then((result) => {
      self.postMessage({ id: requestId, ok: true, result } satisfies WorkerResponse<SimulationWorkerMethodMap>);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({
        id: requestId,
        ok: false,
        error: errorMessage,
      } satisfies WorkerResponse<SimulationWorkerMethodMap>);
    });
};

export {};

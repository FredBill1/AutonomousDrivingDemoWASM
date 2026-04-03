import initWasm, { MpcReferenceTracker, mpc_control_preview } from '../../wasm-core/pkg/wasm_core';

import { createFlatTupleDecoder, decodeFlatTuples, encodeFlatTuples, encodeFlatTuplesToFloat64 } from './flatCodec';
import { disposeWasmResource } from './wasmResource';
import type {
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerSession,
  LocalPlannerUpdateResult,
  LocalPlannerWorkerEventMap,
  LocalPlannerWorkerMethodMap,
  WasmCarState,
  WorkerEvent,
  WorkerHandlerMap,
  WorkerRequest,
  WorkerResponse,
  WasmRuntime,
} from './workerContracts';
import { createCarConfig, createMpcConfig } from './wasmConfig';

const decodePredictedStateQuads = createFlatTupleDecoder<LocalPlannerPathPoint>(4, 'predicted state values', (values, offset) => ({
  x: values[offset],
  y: values[offset + 1],
  yaw: values[offset + 3],
}));

const decodePlannerStateQuads = createFlatTupleDecoder<LocalPlannerReferencePoint>(4, 'planner state values', (values, offset) => ({
  x: values[offset],
  y: values[offset + 1],
  yaw: values[offset + 2],
  velocity: values[offset + 3],
}));

const workerState: {
  runtime: WasmRuntime | null;
  session: LocalPlannerSession | null;
  updateIntervalMs: number;
  mpcTimeStep: number;
} = {
  runtime: null,
  session: null,
  updateIntervalMs: 100,
  mpcTimeStep: 0.07,
};

async function ensureRuntime() {
  if (!workerState.runtime) {
    throw new Error('Local planner worker runtime not initialized');
  }
  return workerState.runtime;
}

function ensureSession() {
  if (!workerState.session) {
    workerState.session = {
      tracker: null,
      latestState: null,
      updateTimerId: null,
      updateInFlight: false,
    };
  }
  if (workerState.session.updateTimerId === null) {
    workerState.session.updateTimerId = setInterval(() => {
      const session = workerState.session;
      if (!session || session.updateInFlight || !session.tracker || !session.latestState) {
        return;
      }
      session.updateInFlight = true;
      void runLocalPlannerUpdate(session.tracker, session.latestState.state, session.latestState.timestamp)
        .then((result) => {
          if (!result || workerState.session !== session) {
            return;
          }
          postEvent('localPlannerUpdate', result);
        })
        .catch((error) => {
          console.error('Failed to update local planner', error);
        })
        .finally(() => {
          if (workerState.session === session) {
            session.updateInFlight = false;
          }
        });
    }, workerState.updateIntervalMs);
  }
  return workerState.session;
}

function clearLocalPlannerTimer() {
  if (!workerState.session || workerState.session.updateTimerId === null) {
    return;
  }
  clearInterval(workerState.session.updateTimerId);
  workerState.session.updateTimerId = null;
}

function postEvent<Key extends keyof LocalPlannerWorkerEventMap>(type: Key, payload: LocalPlannerWorkerEventMap[Key]) {
  self.postMessage({ type, payload } satisfies WorkerEvent<LocalPlannerWorkerEventMap>);
}

function flattenTrajectoryPoints(points: Array<{ x: number; y: number; yaw: number; direction: number }>) {
  return encodeFlatTuples(points, (point) => [point.x, point.y, point.yaw, point.direction]);
}

function decodeControlPairs(flatValues: number[] | Float64Array, timestamp: number, dt: number, initialVelocity: number) {
  const controlSequence: LocalPlannerUpdateResult['controlSequence'] = [];
  let velocity = initialVelocity;
  for (let index = 0; index < flatValues.length; index += 2) {
    velocity += flatValues[index] * dt;
    controlSequence.push({
      timestamp: timestamp + (index / 2) * dt,
      targetVelocity: velocity,
      targetSteer: flatValues[index + 1],
    });
  }
  return controlSequence;
}

async function runLocalPlannerUpdate(
  tracker: MpcReferenceTracker,
  state: WasmCarState,
  timestamp: number,
): Promise<LocalPlannerUpdateResult | null> {
  const { carConfig, mpcConfig } = await ensureRuntime();
  const dt = workerState.mpcTimeStep;
  const referenceResult = tracker.update(state.x, state.y, state.yaw, state.velocity, dt);

  try {
    const referenceStates = referenceResult.reference_states;
    if (referenceStates.length === 0) {
      return null;
    }

    const controlResult = mpc_control_preview(
      mpcConfig,
      carConfig,
      dt,
      referenceResult.model_reference_states,
      state.x,
      state.y,
      state.velocity,
      state.yaw,
      state.steer,
    );

    try {
      return {
        controlSequence: decodeControlPairs(controlResult.controls, timestamp, dt, state.velocity),
        localTrajectory: decodePredictedStateQuads(controlResult.predicted_states).map((point) => ({
          x: point.x,
          y: point.y,
          yaw: point.yaw,
        })),
        referencePoints: decodePlannerStateQuads(referenceStates),
        brakeTrajectory: decodePlannerStateQuads(referenceResult.brake_trajectory),
      };
    } finally {
      controlResult.free();
    }
  } finally {
    referenceResult.free();
  }
}

const handlers: WorkerHandlerMap<LocalPlannerWorkerMethodMap> = {
  async initializeRuntime(payload) {
    await initWasm();
    workerState.runtime = {
      carConfig: createCarConfig(payload.carConfig),
      hybridAStarConfig: null as never,
      mpcConfig: createMpcConfig(payload.mpcConfig),
    };
    workerState.updateIntervalMs = payload.runtime.localPlannerUpdateIntervalMs;
    workerState.mpcTimeStep = payload.runtime.mpcTimeStep;
    return null;
  },

  async setLocalPlannerTrajectory(payload) {
    const session = ensureSession();
    if (!payload.trajectory || payload.trajectory.length === 0) {
      session.tracker?.brake();
      return null;
    }
    const { carConfig, mpcConfig } = await ensureRuntime();
    const nextTracker = new MpcReferenceTracker(
      Float64Array.from(flattenTrajectoryPoints(payload.trajectory)),
      mpcConfig,
      carConfig,
    );
    disposeWasmResource(session.tracker);
    session.tracker = nextTracker;
    return null;
  },

  async setLocalPlannerState(payload) {
    const session = ensureSession();
    session.latestState = {
      state: { ...payload.state },
      timestamp: payload.timestamp,
    };
    return null;
  },

  async brakeLocalPlanner() {
    workerState.session?.tracker?.brake();
    return null;
  },

  async cancelLocalPlanner() {
    if (workerState.session) {
      disposeWasmResource(workerState.session.tracker);
      workerState.session.tracker = null;
    }
    return null;
  },

  async stopLocalPlanner() {
    clearLocalPlannerTimer();
    disposeWasmResource(workerState.session?.tracker);
    workerState.session = null;
    return null;
  },
};

function isKnownRequestType(type: string): type is keyof LocalPlannerWorkerMethodMap {
  return type in handlers;
}

self.onmessage = (event: MessageEvent<WorkerRequest<LocalPlannerWorkerMethodMap> | { id?: number; type?: string; payload?: unknown }>) => {
  const message = event.data;
  if (typeof message.id !== 'number' || typeof message.type !== 'string' || !isKnownRequestType(message.type)) {
    self.postMessage({
      id: typeof message.id === 'number' ? message.id : -1,
      ok: false,
      error: `Unknown worker request: ${String(message.type)}`,
    } satisfies WorkerResponse<LocalPlannerWorkerMethodMap>);
    return;
  }

  void handlers[message.type](message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse<LocalPlannerWorkerMethodMap>);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse<LocalPlannerWorkerMethodMap>);
    });
};

export { decodeFlatTuples, encodeFlatTuplesToFloat64 };
export {};

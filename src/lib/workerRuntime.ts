import initWasm, { CarConfig, MpcConfig } from '../../wasm-core/pkg/wasm_core';

import type { LocalPlannerSession, PlannerSession, SimulationSession, WasmRuntime } from './workerTypes';

export const workerState = {
  initPromise: null as Promise<WasmRuntime> | null,
  nextPlannerToken: 1,
  activePlanner: null as PlannerSession | null,
  simulationSession: null as SimulationSession | null,
  localPlannerSession: null as LocalPlannerSession | null,
};

export async function ensureWasmRuntime(): Promise<WasmRuntime> {
  if (!workerState.initPromise) {
    workerState.initPromise = initWasm().then(() => ({
      carConfig: new CarConfig(),
      mpcConfig: new MpcConfig(),
    }));
  }

  return workerState.initPromise;
}

export async function ensureCarConfig() {
  return (await ensureWasmRuntime()).carConfig;
}

export async function ensureMpcConfig() {
  return (await ensureWasmRuntime()).mpcConfig;
}

export function postEvent(type: string, payload?: unknown) {
  self.postMessage({ type, payload });
}

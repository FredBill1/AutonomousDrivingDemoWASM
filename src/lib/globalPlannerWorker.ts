import initWasm, { HybridAStarPlanner, rs_solve_path } from '../../wasm-core/pkg/wasm_core';

import { decodeFlatTuples } from './flatCodec';
import { disposeWasmResource } from './wasmResource';
import type {
  GlobalPlannerWorkerEventMap,
  GlobalPlannerWorkerMethodMap,
  HybridAStarProgressSegment,
  HybridAStarStartSeedPoint,
  HybridAStarSolution,
  WasmCarState,
  WorkerEvent,
  WorkerHandlerMap,
  WorkerRequest,
  WorkerResponse,
  WasmRuntime,
} from './workerContracts';
import { createCarConfig, createHybridAStarConfig } from './wasmConfig';

const workerState: {
  runtime: WasmRuntime | null;
  activePlanner: { planner: HybridAStarPlanner; cancelled: boolean } | null;
  nextPlannerToken: number;
  hybridAStarStepBudget: number;
  hybridAStarSegmentBatchSize: number;
} = {
  runtime: null,
  activePlanner: null,
  nextPlannerToken: 1,
  hybridAStarStepBudget: 96,
  hybridAStarSegmentBatchSize: 320,
};

async function ensureRuntime() {
  if (!workerState.runtime) {
    throw new Error('Global planner worker runtime not initialized');
  }
  return workerState.runtime;
}

function postEvent<Key extends keyof GlobalPlannerWorkerEventMap>(type: Key, payload: GlobalPlannerWorkerEventMap[Key]) {
  self.postMessage({ type, payload } satisfies WorkerEvent<GlobalPlannerWorkerEventMap>);
}

function decodeExploredSegments(values: ArrayLike<number>): HybridAStarProgressSegment[] {
  return decodeFlatTuples(values, 4, 'explored segments', (items, offset) => ({
    x1: items[offset],
    y1: items[offset + 1],
    x2: items[offset + 2],
    y2: items[offset + 3],
  }));
}

function decodeHybridResult(token: number, result: {
  flat_path: Float64Array | number[];
  explored_segments: Float64Array | number[];
  explored_count: number;
  analytic_expansions: number;
  success: boolean;
}): HybridAStarSolution | null {
  if (!result.success) {
    return null;
  }
  const pathWithDirections = decodeFlatTuples(result.flat_path, 4, 'hybrid path values', (values, offset) => ({
    x: values[offset],
    y: values[offset + 1],
    yaw: values[offset + 2],
    direction: values[offset + 3],
  }));
  return {
    token,
    path: pathWithDirections.map(({ x, y, yaw }) => ({ x, y, yaw })),
    directions: pathWithDirections.map((point) => point.direction),
    exploredSegments: decodeExploredSegments(result.explored_segments),
    exploredCount: result.explored_count,
    analyticExpansions: result.analytic_expansions,
  };
}

function flattenHybridSeedPoints(seed: HybridAStarStartSeedPoint[]) {
  return seed.flatMap((point) => [point.x, point.y, point.yaw, point.velocity]);
}

const handlers: WorkerHandlerMap<GlobalPlannerWorkerMethodMap> = {
  async initializeRuntime(payload) {
    await initWasm();
    workerState.runtime = {
      carConfig: createCarConfig(payload.carConfig),
      hybridAStarConfig: createHybridAStarConfig(payload.hybridAStarConfig),
      mpcConfig: null as never,
    };
    workerState.hybridAStarStepBudget = payload.runtime.hybridAStarStepBudget;
    workerState.hybridAStarSegmentBatchSize = payload.runtime.hybridAStarSegmentBatchSize;
    return null;
  },

  async solveHybridAStar(payload) {
    const { carConfig, hybridAStarConfig } = await ensureRuntime();
    disposeWasmResource(workerState.activePlanner?.planner);
    const planner = payload.startIsTrajectorySeed
      ? HybridAStarPlanner.from_trajectory_seed(
          Float64Array.from(flattenHybridSeedPoints(payload.start as HybridAStarStartSeedPoint[])),
          payload.goal.x,
          payload.goal.y,
          payload.goal.yaw,
          payload.obstacleCoordinates,
          payload.maxIterations,
          carConfig,
          hybridAStarConfig,
        )
      : new HybridAStarPlanner(
          (payload.start as WasmCarState).x,
          (payload.start as WasmCarState).y,
          (payload.start as WasmCarState).yaw,
          payload.goal.x,
          payload.goal.y,
          payload.goal.yaw,
          payload.obstacleCoordinates,
          payload.maxIterations,
          carConfig,
          hybridAStarConfig,
        );

    workerState.activePlanner = { planner, cancelled: false };
    const plannerToken = payload.requestToken ?? workerState.nextPlannerToken;
    if (payload.requestToken === undefined) {
      workerState.nextPlannerToken += 1;
    }
    const plannerSession = workerState.activePlanner;

    while (!plannerSession.cancelled) {
      const finished = plannerSession.planner.step(workerState.hybridAStarStepBudget);
      const exploredFlat = plannerSession.planner.take_explored_segments();
      if (exploredFlat.length > 0) {
        const decoded = decodeExploredSegments(exploredFlat);
        for (let index = 0; index < decoded.length; index += workerState.hybridAStarSegmentBatchSize) {
          postEvent('hybridAStarProgress', {
            token: plannerToken,
            segments: decoded.slice(index, index + workerState.hybridAStarSegmentBatchSize),
            exploredCount: plannerSession.planner.explored_count,
            analyticExpansions: plannerSession.planner.analytic_expansions,
          });
        }
      }

      if (finished) {
        const result = plannerSession.planner.take_result();
        disposeWasmResource(plannerSession.planner);
        workerState.activePlanner = null;
        if (!result) {
          return null;
        }
        try {
          return decodeHybridResult(plannerToken, result);
        } finally {
          result.free();
        }
      }

      await new Promise<void>((resolve) => setTimeout(resolve, 0));
    }

    disposeWasmResource(plannerSession.planner);
    workerState.activePlanner = null;
    throw new Error('Hybrid A* search cancelled');
  },

  async cancelHybridAStar() {
    if (workerState.activePlanner) {
      workerState.activePlanner.cancelled = true;
    }
    return null;
  },

  async solveReedsSheppCandidates(payload) {
    await ensureRuntime();
    const solutions: GlobalPlannerWorkerMethodMap['solveReedsSheppCandidates']['result'] = [];
    for (const turnRadius of payload.turnRadii) {
      for (const runwayLength of payload.runwayLengths) {
        try {
          const solvedPath = rs_solve_path(
            payload.start.x,
            payload.start.y,
            payload.start.yaw,
            payload.goal.x,
            payload.goal.y,
            payload.goal.yaw,
            turnRadius,
            runwayLength,
            payload.stepSize,
            payload.lengthTolerance,
          );
          try {
            solutions.push({
              path: decodeFlatTuples(solvedPath.flat_coordinates(), 3, 'flat coordinates', (values, offset) => ({
                x: values[offset],
                y: values[offset + 1],
                yaw: values[offset + 2],
              })),
              totalLength: solvedPath.total_length(),
              segmentCount: solvedPath.segment_count(),
              runwayLength: solvedPath.runway_length(),
              turnRadius: solvedPath.turn_radius(),
            });
          } finally {
            solvedPath.free();
          }
        } catch (error) {
          const reason = error instanceof Error ? error.message : String(error);
          console.warn(`Ignoring invalid Reeds-Shepp candidate (${reason})`, { turnRadius, runwayLength });
        }
      }
    }

    solutions.sort((left, right) => {
      if (Math.abs(left.totalLength - right.totalLength) < payload.lengthTolerance) {
        return left.segmentCount - right.segmentCount;
      }
      return left.totalLength - right.totalLength;
    });
    return solutions;
  },
};

function isKnownRequestType(type: string): type is keyof GlobalPlannerWorkerMethodMap {
  return type in handlers;
}

self.onmessage = (event: MessageEvent<WorkerRequest<GlobalPlannerWorkerMethodMap> | { id?: number; type?: string; payload?: unknown }>) => {
  const message = event.data;
  if (typeof message.id !== 'number' || typeof message.type !== 'string' || !isKnownRequestType(message.type)) {
    self.postMessage({
      id: typeof message.id === 'number' ? message.id : -1,
      ok: false,
      error: `Unknown worker request: ${String(message.type)}`,
    } satisfies WorkerResponse<GlobalPlannerWorkerMethodMap>);
    return;
  }

  void handlers[message.type](message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse<GlobalPlannerWorkerMethodMap>);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse<GlobalPlannerWorkerMethodMap>);
    });
};

export {};

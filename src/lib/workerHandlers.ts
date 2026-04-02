import { HybridAStarPlanner } from '../../wasm-core/pkg/wasm_core';

import { disposeWasmResource } from './wasmResource';
import { decodeExploredSegments, flattenHybridSeedPoints, snapshotHybridResult } from './workerCodecs';
import { postEvent, workerState, ensureWasmRuntime } from './workerRuntime';
import {
  HYBRID_ASTAR_STEP_BUDGET,
  HYBRID_SEGMENT_BATCH_SIZE,
  type HybridSeedPoint,
  type WasmCarState,
} from './workerTypes';

export async function solveHybridAStar(payload: {
  start: WasmCarState | HybridSeedPoint[];
  startIsTrajectorySeed?: boolean;
  goal: WasmCarState;
  obstacleCoordinates: number[];
  maxIterations: number;
  requestToken?: number;
}) {
  await ensureWasmRuntime();

  disposeWasmResource(workerState.activePlanner?.planner);
  const planner = payload.startIsTrajectorySeed
    ? HybridAStarPlanner.from_trajectory_seed(
        Float64Array.from(flattenHybridSeedPoints(payload.start as HybridSeedPoint[])),
        payload.goal.x,
        payload.goal.y,
        payload.goal.yaw,
        Float64Array.from(payload.obstacleCoordinates),
        payload.maxIterations,
      )
    : new HybridAStarPlanner(
        (payload.start as WasmCarState).x,
        (payload.start as WasmCarState).y,
        (payload.start as WasmCarState).yaw,
        payload.goal.x,
        payload.goal.y,
        payload.goal.yaw,
        Float64Array.from(payload.obstacleCoordinates),
        payload.maxIterations,
      );
  workerState.activePlanner = {
    planner,
    cancelled: false,
  };

  const plannerToken = payload.requestToken ?? workerState.nextPlannerToken;
  if (payload.requestToken === undefined) {
    workerState.nextPlannerToken += 1;
  }
  const plannerSession = workerState.activePlanner;

  while (!plannerSession.cancelled) {
    const finished = plannerSession.planner.step(HYBRID_ASTAR_STEP_BUDGET);
    const exploredFlat = plannerSession.planner.take_explored_segments();
    if (exploredFlat.length > 0) {
      const decoded = decodeExploredSegments(exploredFlat);
      for (let index = 0; index < decoded.length; index += HYBRID_SEGMENT_BATCH_SIZE) {
        postEvent('hybridAStarProgress', {
          token: plannerToken,
          segments: decoded.slice(index, index + HYBRID_SEGMENT_BATCH_SIZE),
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
      return snapshotHybridResult(plannerToken, result);
    }

    await new Promise<void>((resolve) => setTimeout(resolve, 0));
  }

  disposeWasmResource(plannerSession.planner);
  workerState.activePlanner = null;
  throw new Error('Hybrid A* search cancelled');
}

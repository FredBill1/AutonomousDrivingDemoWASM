import { type CarConfig, MpcReferenceTracker, mpc_control_preview, trajectory_check_collision } from '../../wasm-core/pkg/wasm_core';

import {
  createFlatTupleDecoder,
  decodeFlatTuples,
  encodeFlatTuples,
  encodeFlatTuplesToFloat64,
} from './flatCodec';
import { ensureWasmRuntime } from './workerRuntime';
import type {
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerUpdateResult,
  TrackingPlan,
  WasmCarState,
} from './workerTypes';
import { MPC_DT } from './workerTypes';

export function buildTrajectoryCollisionInput(plan: TrackingPlan | null) {
  if (!plan || plan.path.length === 0) {
    return null;
  }
  return encodeFlatTuplesToFloat64(plan.path, (point) => [point.x, point.y, point.yaw]);
}

export function checkTrajectoryCollision(
  config: CarConfig,
  plan: TrackingPlan | null,
  obstacleCoordinates: ArrayLike<number>,
) {
  const trajectory = buildTrajectoryCollisionInput(plan);
  if (!trajectory || obstacleCoordinates.length < 2) {
    return false;
  }
  return trajectory_check_collision(config, trajectory, Float64Array.from(obstacleCoordinates));
}

export const decodeFlatCoordinates = createFlatTupleDecoder(3, 'flat coordinates', (values, offset) => ({
  x: values[offset],
  y: values[offset + 1],
  yaw: values[offset + 2],
}));

export function flattenTrajectoryPoints(points: Array<{ x: number; y: number; yaw: number; direction: number }>) {
  return encodeFlatTuples(points, (point) => [point.x, point.y, point.yaw, point.direction]);
}

export const decodePredictedStateQuads = createFlatTupleDecoder<LocalPlannerPathPoint>(
  4,
  'predicted state values',
  (values, offset) => ({
    x: values[offset],
    y: values[offset + 1],
    yaw: values[offset + 3],
  }),
);

export const decodePlannerStateQuads = createFlatTupleDecoder<LocalPlannerReferencePoint>(4, 'planner state values', (values, offset) => ({
  x: values[offset],
  y: values[offset + 1],
  yaw: values[offset + 2],
  velocity: values[offset + 3],
}));

export function decodeControlPairs(
  flatValues: number[] | Float64Array,
  timestamp: number,
  dt: number,
  initialVelocity: number,
) {
  const controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> = [];
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

export function runLocalPlannerUpdate(
  tracker: MpcReferenceTracker,
  state: WasmCarState,
  timestamp: number,
): Promise<LocalPlannerUpdateResult | null> {
  return ensureWasmRuntime().then(({ carConfig, mpcConfig }) => {
    const dt = MPC_DT;
    const referenceResult = tracker.update(state.x, state.y, state.yaw, state.velocity, dt);

    try {
      const referenceStates = referenceResult.reference_states;
      if (referenceStates.length === 0) {
        return null;
      }

      const brakeTrajectory = referenceResult.brake_trajectory;
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
          brakeTrajectory: decodePlannerStateQuads(brakeTrajectory),
        };
      } finally {
        controlResult.free();
      }
    } finally {
      referenceResult.free();
    }
  });
}

export function decodeHybridResult(result: {
  token: number;
  flat_path: Float64Array | number[];
  explored_segments: Float64Array | number[];
  explored_count: number;
  analytic_expansions: number;
  success: boolean;
}) {
  if (!result.success) {
    return null;
  }

  const pathValues = result.flat_path;
  const exploredValues = result.explored_segments;

  const pathWithDirections = decodeFlatTuples(pathValues, 4, 'hybrid path values', (values, offset) => ({
    x: values[offset],
    y: values[offset + 1],
    yaw: values[offset + 2],
    direction: values[offset + 3],
  }));
  const path = pathWithDirections.map(({ x, y, yaw }) => ({ x, y, yaw }));
  const directions = pathWithDirections.map((point) => point.direction);
  const exploredSegments = decodeExploredSegments(exploredValues);

  return {
    token: result.token,
    path,
    directions,
    exploredSegments,
    exploredCount: result.explored_count,
    analyticExpansions: result.analytic_expansions,
  };
}

export function snapshotHybridResult(
  token: number,
  result: {
    flat_path: Float64Array;
    explored_segments: Float64Array;
    explored_count: number;
    analytic_expansions: number;
    success: boolean;
    free?: () => void;
  },
) {
  try {
    return decodeHybridResult({
      token,
      flat_path: result.flat_path,
      explored_segments: result.explored_segments,
      explored_count: result.explored_count,
      analytic_expansions: result.analytic_expansions,
      success: result.success,
    });
  } finally {
    result.free?.();
  }
}

export function flattenHybridSeedPoints(seed: { x: number; y: number; yaw: number; velocity: number }[]) {
  return encodeFlatTuples(seed, (point) => [point.x, point.y, point.yaw, point.velocity]);
}

export const decodeExploredSegments = createFlatTupleDecoder(4, 'explored segments', (values, offset) => ({
  x1: values[offset],
  y1: values[offset + 1],
  x2: values[offset + 2],
  y2: values[offset + 3],
}));

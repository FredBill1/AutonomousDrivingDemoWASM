import {
  CarConfig,
  MpcConfig,
  MpcReferenceTracker,
  mpc_control_preview,
  trajectory_check_collision,
} from '../../wasm-core/pkg/wasm_core';

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
  return Float64Array.from(plan.path.flatMap((point) => [point.x, point.y, point.yaw]));
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

type FlatArray = ArrayLike<number>;

function decodeFlatValues<T>(values: FlatArray, stride: number, map: (source: FlatArray, index: number) => T) {
  const decoded: T[] = [];
  for (let index = 0; index < values.length; index += stride) {
    decoded.push(map(values, index));
  }
  return decoded;
}

export function decodeFlatCoordinates(flatCoordinates: Float64Array): Array<{ x: number; y: number; yaw: number }> {
  return decodeFlatValues(flatCoordinates, 3, (coords, index) => ({
    x: coords[index],
    y: coords[index + 1],
    yaw: coords[index + 2],
  }));
}

export function flattenTrajectoryPoints(points: Array<{ x: number; y: number; yaw: number; direction: number }>) {
  return points.flatMap((point) => [point.x, point.y, point.yaw, point.direction]);
}

export function decodePredictedStateQuads(flatValues: number[] | Float64Array): LocalPlannerPathPoint[] {
  return decodeFlatValues(flatValues, 4, (values, index) => ({
    x: values[index],
    y: values[index + 1],
    yaw: values[index + 3],
  }));
}

export function decodePlannerStateQuads(flatValues: number[] | Float64Array): LocalPlannerReferencePoint[] {
  return decodeFlatValues(flatValues, 4, (values, index) => ({
    x: values[index],
    y: values[index + 1],
    yaw: values[index + 2],
    velocity: values[index + 3],
  }));
}

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
  const mpcConfig = new MpcConfig();
  const carConfig = new CarConfig();
  const dt = MPC_DT;
  const referenceResult = tracker.update(state.x, state.y, state.yaw, state.velocity, dt);
  const referenceStates = referenceResult.reference_states;
  const modelReferenceStates = referenceResult.model_reference_states;
  if (referenceStates.length === 0) {
    referenceResult.free();
    mpcConfig.free();
    carConfig.free();
    return Promise.resolve(null);
  }

  const brakeTrajectory = referenceResult.brake_trajectory;
  const controlResult = mpc_control_preview(
    mpcConfig,
    carConfig,
    dt,
    modelReferenceStates,
    state.x,
    state.y,
    state.velocity,
    state.yaw,
    state.steer,
  );
  mpcConfig.free();
  carConfig.free();
  try {
    return Promise.resolve({
      controlSequence: decodeControlPairs(controlResult.controls, timestamp, dt, state.velocity),
      localTrajectory: decodePredictedStateQuads(controlResult.predicted_states).map((point) => ({
        x: point.x,
        y: point.y,
        yaw: point.yaw,
      })),
      referencePoints: decodePlannerStateQuads(referenceStates),
      brakeTrajectory: decodePlannerStateQuads(brakeTrajectory),
    });
  } finally {
    controlResult.free();
    referenceResult.free();
  }
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

  const pathPoints = decodeFlatValues(pathValues, 4, (values, index) => ({
    x: values[index],
    y: values[index + 1],
    yaw: values[index + 2],
    direction: values[index + 3],
  }));
  const path = pathPoints.map(({ direction: _direction, ...rest }) => rest);
  const directions = pathPoints.map((point) => point.direction);
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
  return seed.flatMap((point) => [point.x, point.y, point.yaw, point.velocity]);
}

export function decodeExploredSegments(flatSegments: Float64Array | number[]) {
  return decodeFlatValues(flatSegments, 4, (values, index) => ({
    x1: values[index],
    y1: values[index + 1],
    x2: values[index + 2],
    y2: values[index + 3],
  }));
}

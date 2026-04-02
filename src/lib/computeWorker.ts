import {
  CarConfig,
  CarState,
  MpcConfig,
  MpcReferenceTracker,
  path_check_collision,
  rs_solve_path,
} from '../../wasm-core/pkg/wasm_core';

import { encodeFlatTuplesToFloat64 } from './flatCodec';
import { disposeWasmResource, usingWasmPair, usingWasmResource } from './wasmResource';
import { checkTrajectoryCollision, decodeFlatCoordinates, flattenTrajectoryPoints } from './workerCodecs';
import { solveHybridAStar } from './workerHandlers';
import {
  applySimulationStop,
  clearLocalPlannerTimer,
  clearSimulationTimers,
  computeStepCarState,
  DEFAULT_PUBLISH_INTERVAL_MS,
  DEFAULT_SIM_DELTA_TIME,
  DEFAULT_SIM_INTERVAL_MS,
  ensureLocalPlannerSession,
  ensureWasmCore,
  setSimulationControlSequenceInternal,
  startSimulationLoop,
  workerState,
} from './workerHelpers';
import {
  type LocalPlannerControlPoint,
  type WasmCarState,
  type WorkerRequest,
  type WorkerResponse,
} from './workerTypes';

function createSimulationSession(state: WasmCarState, timestamp = 0, simDeltaTime = DEFAULT_SIM_DELTA_TIME) {
  return {
    state,
    timestamp,
    simDeltaTime,
    controlSequence: null,
    stopped: true,
    simulationTimerId: null,
    publishTimerId: null,
    loopToken: 0,
    stateVersion: 0,
  };
}

function requireSimulationSession() {
  if (!workerState.simulationSession) {
    throw new Error('Simulation not initialized');
  }
  return workerState.simulationSession;
}

function replaceLocalPlannerTracker(tracker: MpcReferenceTracker | null) {
  disposeWasmResource(workerState.localPlannerSession?.tracker);
  if (workerState.localPlannerSession) {
    workerState.localPlannerSession.tracker = tracker;
  }
}

function solveReedsSheppCandidate(payload: {
  start: WasmCarState;
  goal: WasmCarState;
  turnRadius: number;
  runwayLength: number;
  stepSize: number;
  lengthTolerance: number;
}) {
  return usingWasmResource(
    rs_solve_path(
      payload.start.x,
      payload.start.y,
      payload.start.yaw,
      payload.goal.x,
      payload.goal.y,
      payload.goal.yaw,
      payload.turnRadius,
      payload.runwayLength,
      payload.stepSize,
      payload.lengthTolerance,
    ),
    (solvedPath) => ({
      path: decodeFlatCoordinates(solvedPath.flat_coordinates()),
      totalLength: solvedPath.total_length(),
      segmentCount: solvedPath.segment_count(),
      runwayLength: solvedPath.runway_length(),
      turnRadius: solvedPath.turn_radius(),
    }),
  );
}

const handlers = {
  async getCarConfigSnapshot() {
    const config = await ensureWasmCore();
    return {
      wheelBase: config.wheel_base,
      length: config.length,
      width: config.width,
      backToWheel: config.back_to_wheel,
      wheelLength: config.wheel_length,
      wheelWidth: config.wheel_width,
      wheelSpacing: config.wheel_spacing,
      backToCenter: config.back_to_center,
      collisionLength: config.collision_length,
      collisionWidth: config.collision_width,
      collisionRadius: config.collision_radius,
      targetMaxSteer: config.target_max_steer,
      maxSteer: config.max_steer,
      maxSteerSpeed: config.max_steer_speed,
      maxSpeed: config.max_speed,
      minSpeed: config.min_speed,
      maxAccel: config.max_accel,
      maxCentripetalAccel: config.max_centripetal_accel,
      targetSpeed: config.target_speed,
      targetMinTurningRadius: config.target_min_turning_radius,
      scanRadius: config.scan_radius,
    };
  },

  stepCarState(payload: { current: WasmCarState; targetVelocity: number; targetSteer: number; dt: number }) {
    const { current, targetVelocity, targetSteer, dt } = payload;
    return computeStepCarState(current, targetVelocity, targetSteer, dt);
  },

  async initSimulation(payload: {
    state: WasmCarState;
    timestamp?: number;
    simDeltaTime?: number;
    simulationIntervalMs?: number;
    publishIntervalMs?: number;
  }) {
    await ensureWasmCore();
    workerState.simulationSession = createSimulationSession(
      payload.state,
      payload.timestamp ?? 0,
      payload.simDeltaTime ?? DEFAULT_SIM_DELTA_TIME,
    );
    startSimulationLoop(
      payload.simulationIntervalMs ?? DEFAULT_SIM_INTERVAL_MS,
      payload.publishIntervalMs ?? DEFAULT_PUBLISH_INTERVAL_MS,
    );
    return null;
  },

  async setSimulationState(payload: { state: WasmCarState; timestamp?: number }) {
    await ensureWasmCore();
    if (!workerState.simulationSession) {
      workerState.simulationSession = createSimulationSession(payload.state, payload.timestamp ?? 0);
      startSimulationLoop(DEFAULT_SIM_INTERVAL_MS, DEFAULT_PUBLISH_INTERVAL_MS);
    } else {
      workerState.simulationSession.state = payload.state;
      workerState.simulationSession.controlSequence = null;
      workerState.simulationSession.stopped = true;
      workerState.simulationSession.stateVersion += 1;
      if (payload.timestamp !== undefined) {
        workerState.simulationSession.timestamp = payload.timestamp;
      }
    }
    return null;
  },

  setSimulationControlSequence(payload: { controlSequence: LocalPlannerControlPoint[] }) {
    setSimulationControlSequenceInternal(requireSimulationSession(), payload.controlSequence);
    return Promise.resolve(null);
  },

  stopSimulationMotion() {
    applySimulationStop(requireSimulationSession());
    return Promise.resolve(null);
  },

  setLocalPlannerTrajectory(payload: {
    trajectory: Array<{ x: number; y: number; yaw: number; direction: number }> | null;
  }) {
    const session = ensureLocalPlannerSession();

    if (!payload.trajectory || payload.trajectory.length === 0) {
      session.tracker?.brake();
      return Promise.resolve(null);
    }

    session.tracker = usingWasmPair(new MpcConfig(), new CarConfig(), (mpcConfig, carConfig) => {
      disposeWasmResource(session.tracker);
      return new MpcReferenceTracker(
        Float64Array.from(flattenTrajectoryPoints(payload.trajectory)),
        mpcConfig,
        carConfig,
      );
    });
    return Promise.resolve(null);
  },

  setLocalPlannerState(payload: { state: WasmCarState; timestamp: number; updateIntervalMs?: number }) {
    const session = ensureLocalPlannerSession();
    session.latestState = {
      state: payload.state,
      timestamp: payload.timestamp,
    };
    if (payload.updateIntervalMs !== undefined && payload.updateIntervalMs !== session.updateIntervalMs) {
      session.updateIntervalMs = payload.updateIntervalMs;
      clearLocalPlannerTimer();
      ensureLocalPlannerSession();
    }
    return Promise.resolve(null);
  },

  brakeLocalPlanner() {
    workerState.localPlannerSession?.tracker?.brake();
    return Promise.resolve(null);
  },

  cancelLocalPlanner() {
    if (workerState.localPlannerSession) {
      replaceLocalPlannerTracker(null);
    }
    return Promise.resolve(null);
  },

  resumeSimulationMotion() {
    requireSimulationSession().stopped = false;
    return Promise.resolve(null);
  },

  stopSimulation() {
    clearSimulationTimers();
    workerState.simulationSession = null;
    clearLocalPlannerTimer();
    disposeWasmResource(workerState.localPlannerSession?.tracker);
    workerState.localPlannerSession = null;
    return Promise.resolve(null);
  },

  async checkCollision(payload: { state: WasmCarState; obstacleCoordinates: number[] }) {
    const config = await ensureWasmCore();
    const { state: stateLike, obstacleCoordinates } = payload;
    return usingWasmResource(
      new CarState(stateLike.x, stateLike.y, stateLike.yaw, stateLike.velocity, stateLike.steer),
      (state) => state.check_collision(config, Float64Array.from(obstacleCoordinates)),
    );
  },

  async checkPathCollision(payload: {
    path: Array<{ x: number; y: number; yaw: number }>;
    obstacleCoordinates: number[];
  }) {
    const config = await ensureWasmCore();
    return path_check_collision(
      config,
      encodeFlatTuplesToFloat64(payload.path, (point) => [point.x, point.y, point.yaw]),
      Float64Array.from(payload.obstacleCoordinates),
    );
  },

  async checkTrajectoryCollision(payload: {
    path: Array<{ x: number; y: number; yaw: number }>;
    obstacleCoordinates: number[];
  }) {
    const config = await ensureWasmCore();
    return checkTrajectoryCollision(config, { path: payload.path, directions: [] }, payload.obstacleCoordinates);
  },

  async solveReedsSheppCandidates(payload: {
    start: WasmCarState;
    goal: WasmCarState;
    turnRadii: number[];
    runwayLengths: number[];
    stepSize: number;
    lengthTolerance: number;
  }) {
    await ensureWasmCore();

    const solutions: Array<{
      path: Array<{ x: number; y: number; yaw: number }>;
      totalLength: number;
      segmentCount: number;
      runwayLength: number;
      turnRadius: number;
    }> = [];

    for (const turnRadius of payload.turnRadii) {
      for (const runwayLength of payload.runwayLengths) {
        try {
          solutions.push(
            solveReedsSheppCandidate({
              start: payload.start,
              goal: payload.goal,
              turnRadius,
              runwayLength,
              stepSize: payload.stepSize,
              lengthTolerance: payload.lengthTolerance,
            }),
          );
        } catch {
          // Ignore invalid combinations.
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

  solveHybridAStar,

  cancelHybridAStar() {
    if (workerState.activePlanner) {
      workerState.activePlanner.cancelled = true;
    }
    return Promise.resolve(null);
  },
} as const;

self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const message = event.data;
  const handler = handlers[message.type as keyof typeof handlers] as ((payload: never) => Promise<unknown>) | undefined;

  if (!handler) {
    self.postMessage({
      id: message.id,
      ok: false,
      error: `Unknown worker request: ${message.type}`,
    } satisfies WorkerResponse);
    return;
  }

  void handler(message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse);
    });
};

export {};

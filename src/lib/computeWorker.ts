import {
  CarConfig,
  CarState,
  MpcConfig,
  MpcReferenceTracker,
  path_check_collision,
  rs_solve_path,
} from '../../wasm-core/pkg/wasm_core';

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

const withCarConfig = async <T>(fn: (config: CarConfig) => Promise<T> | T) => {
  const config = await ensureWasmCore();
  return fn(config);
};

const withCarConfigFor = async <Payload, Result>(
  payload: Payload,
  handler: (config: CarConfig, payload: Payload) => Result | Promise<Result>,
) => withCarConfig((config) => handler(config, payload));

const handlePathCollision = (
  config: CarConfig,
  payload: { path: Array<{ x: number; y: number; yaw: number }>; obstacleCoordinates: number[] },
) => {
  const flatPath = payload.path.flatMap((point) => [point.x, point.y, point.yaw]);
  return path_check_collision(config, Float64Array.from(flatPath), Float64Array.from(payload.obstacleCoordinates));
};

const handleTrajectoryCollision = (
  config: CarConfig,
  payload: { path: Array<{ x: number; y: number; yaw: number }>; obstacleCoordinates: number[] },
) => checkTrajectoryCollision(config, { path: payload.path, directions: [] }, payload.obstacleCoordinates);

const handlers = {
  async getCarConfigSnapshot() {
    return withCarConfig((config) => ({
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
    }));
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
    workerState.simulationSession = {
      state: payload.state,
      timestamp: payload.timestamp ?? 0,
      simDeltaTime: payload.simDeltaTime ?? DEFAULT_SIM_DELTA_TIME,
      controlSequence: null,
      stopped: true,
      simulationTimerId: null,
      publishTimerId: null,
      loopToken: 0,
      stateVersion: 0,
    };
    startSimulationLoop(
      payload.simulationIntervalMs ?? DEFAULT_SIM_INTERVAL_MS,
      payload.publishIntervalMs ?? DEFAULT_PUBLISH_INTERVAL_MS,
    );
    return null;
  },

  async setSimulationState(payload: { state: WasmCarState; timestamp?: number }) {
    await ensureWasmCore();
    if (!workerState.simulationSession) {
      workerState.simulationSession = {
        state: payload.state,
        timestamp: payload.timestamp ?? 0,
        simDeltaTime: DEFAULT_SIM_DELTA_TIME,
        controlSequence: null,
        stopped: true,
        simulationTimerId: null,
        publishTimerId: null,
        loopToken: 0,
        stateVersion: 0,
      };
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
    if (!workerState.simulationSession) {
      return Promise.reject(new Error('Simulation not initialized'));
    }
    setSimulationControlSequenceInternal(workerState.simulationSession, payload.controlSequence);
    return Promise.resolve(null);
  },

  stopSimulationMotion() {
    if (!workerState.simulationSession) {
      return Promise.reject(new Error('Simulation not initialized'));
    }
    applySimulationStop(workerState.simulationSession);
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

    session.tracker?.free();
    const mpcConfig = new MpcConfig();
    const carConfig = new CarConfig();
    try {
      session.tracker = new MpcReferenceTracker(
        Float64Array.from(flattenTrajectoryPoints(payload.trajectory)),
        mpcConfig,
        carConfig,
      );
    } finally {
      mpcConfig.free();
      carConfig.free();
    }
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
      workerState.localPlannerSession.tracker?.free();
      workerState.localPlannerSession.tracker = null;
    }
    return Promise.resolve(null);
  },

  resumeSimulationMotion() {
    if (!workerState.simulationSession) {
      return Promise.reject(new Error('Simulation not initialized'));
    }
    workerState.simulationSession.stopped = false;
    return Promise.resolve(null);
  },

  stopSimulation() {
    clearSimulationTimers();
    workerState.simulationSession = null;
    clearLocalPlannerTimer();
    workerState.localPlannerSession?.tracker?.free();
    workerState.localPlannerSession = null;
    return Promise.resolve(null);
  },

  async checkCollision(payload: { state: WasmCarState; obstacleCoordinates: number[] }) {
    return withCarConfig((config) => {
      const { state: stateLike, obstacleCoordinates } = payload;
      const state = new CarState(stateLike.x, stateLike.y, stateLike.yaw, stateLike.velocity, stateLike.steer);
      try {
        return state.check_collision(config, Float64Array.from(obstacleCoordinates));
      } finally {
        state.free();
      }
    });
  },

  checkPathCollision: (payload: {
    path: Array<{ x: number; y: number; yaw: number }>;
    obstacleCoordinates: number[];
  }) => withCarConfigFor(payload, handlePathCollision),

  checkTrajectoryCollision: (payload: {
    path: Array<{ x: number; y: number; yaw: number }>;
    obstacleCoordinates: number[];
  }) => withCarConfigFor(payload, handleTrajectoryCollision),

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
              path: decodeFlatCoordinates(solvedPath.flat_coordinates()),
              totalLength: solvedPath.total_length(),
              segmentCount: solvedPath.segment_count(),
              runwayLength: solvedPath.runway_length(),
              turnRadius: solvedPath.turn_radius(),
            });
          } finally {
            solvedPath.free();
          }
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

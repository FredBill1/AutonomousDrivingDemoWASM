import { CarState, MpcReferenceTracker, path_check_collision, rs_solve_path } from '../../wasm-core/pkg/wasm_core';

import { encodeFlatTuplesToFloat64 } from './flatCodec';
import { disposeWasmResource } from './wasmResource';
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
  setSimulationControlSequenceInternal,
  startSimulationLoop,
} from './workerHelpers';
import { ensureCarConfig, ensureWasmRuntime, workerState } from './workerRuntime';
import type {
  LocalPlannerControlPoint,
  LocalPlannerTrajectoryPoint,
  SimulationSession,
  WasmCarState,
  WasmConfigSnapshot,
  WorkerHandlerMap,
  WorkerMethodMap,
} from './workerTypes';

function createSimulationSession(
  state: WasmCarState,
  timestamp = 0,
  simDeltaTime = DEFAULT_SIM_DELTA_TIME,
): SimulationSession {
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

function toCarConfigSnapshot(config: Awaited<ReturnType<typeof ensureCarConfig>>): WasmConfigSnapshot {
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
}

async function createTracker(trajectory: LocalPlannerTrajectoryPoint[]) {
  const { carConfig, mpcConfig } = await ensureWasmRuntime();
  return new MpcReferenceTracker(Float64Array.from(flattenTrajectoryPoints(trajectory)), mpcConfig, carConfig);
}

type ReedsSheppCandidate = WorkerMethodMap['solveReedsSheppCandidates']['result'][number];

export const handlers: WorkerHandlerMap = {
  async getCarConfigSnapshot() {
    return toCarConfigSnapshot(await ensureCarConfig());
  },

  async stepCarState(payload) {
    const { current, targetVelocity, targetSteer, dt } = payload;
    return computeStepCarState(current, targetVelocity, targetSteer, dt);
  },

  async initSimulation(payload) {
    await ensureWasmRuntime();
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

  async setSimulationState(payload) {
    await ensureWasmRuntime();
    if (!workerState.simulationSession) {
      workerState.simulationSession = createSimulationSession(payload.state, payload.timestamp ?? 0);
      startSimulationLoop(DEFAULT_SIM_INTERVAL_MS, DEFAULT_PUBLISH_INTERVAL_MS);
      return null;
    }

    workerState.simulationSession.state = payload.state;
    workerState.simulationSession.controlSequence = null;
    workerState.simulationSession.stopped = true;
    workerState.simulationSession.stateVersion += 1;
    if (payload.timestamp !== undefined) {
      workerState.simulationSession.timestamp = payload.timestamp;
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

  async setLocalPlannerTrajectory(payload) {
    const session = ensureLocalPlannerSession();
    if (!payload.trajectory || payload.trajectory.length === 0) {
      session.tracker?.brake();
      return null;
    }

    const nextTracker = await createTracker(payload.trajectory);
    disposeWasmResource(session.tracker);
    session.tracker = nextTracker;
    return null;
  },

  setLocalPlannerState(payload) {
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

  async checkCollision(payload) {
    const config = await ensureCarConfig();
    const state = new CarState(
      payload.state.x,
      payload.state.y,
      payload.state.yaw,
      payload.state.velocity,
      payload.state.steer,
    );

    try {
      return state.check_collision(config, Float64Array.from(payload.obstacleCoordinates));
    } finally {
      state.free();
    }
  },

  async checkPathCollision(payload) {
    return path_check_collision(
      await ensureCarConfig(),
      encodeFlatTuplesToFloat64(payload.path, (point) => [point.x, point.y, point.yaw]),
      Float64Array.from(payload.obstacleCoordinates),
    );
  },

  async checkTrajectoryCollision(payload) {
    return checkTrajectoryCollision(
      await ensureCarConfig(),
      { path: payload.path, directions: [] },
      payload.obstacleCoordinates,
    );
  },

  async solveReedsSheppCandidates(payload) {
    await ensureWasmRuntime();

    const solutions: ReedsSheppCandidate[] = [];
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
        } catch (error) {
          const reason = error instanceof Error ? error.message : String(error);
          console.warn(`Ignoring invalid Reeds-Shepp candidate (${reason})`, { turnRadius, runwayLength });
        }
      }
    }

    solutions.sort((left: ReedsSheppCandidate, right: ReedsSheppCandidate) => {
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
} satisfies WorkerHandlerMap;

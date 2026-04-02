import initWasm, { CarConfig, CarState } from '../../wasm-core/pkg/wasm_core';

import { runLocalPlannerUpdate } from './workerCodecs';
import {
  DEFAULT_LOCAL_PLANNER_DT,
  DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS,
  DEFAULT_PUBLISH_INTERVAL_MS,
  DEFAULT_SIM_DELTA_TIME,
  DEFAULT_SIM_INTERVAL_MS,
  type LocalPlannerControlPoint,
  type LocalPlannerSession,
  type PlannerSession,
  type SimulationSession,
  type WasmCarState,
  type WorkerEvent,
} from './workerTypes';

export const workerState = {
  initPromise: null as Promise<CarConfig> | null,
  nextPlannerToken: 1,
  activePlanner: null as PlannerSession | null,
  simulationSession: null as SimulationSession | null,
  localPlannerSession: null as LocalPlannerSession | null,
};

export async function ensureWasmCore() {
  if (!workerState.initPromise) {
    workerState.initPromise = initWasm().then(() => new CarConfig());
  }
  return workerState.initPromise;
}

export function postEvent(type: string, payload?: unknown) {
  self.postMessage({ type, payload } satisfies WorkerEvent);
}

export function emitSimulationState() {
  if (!workerState.simulationSession) return;
  postEvent('simulationState', {
    timestamp: workerState.simulationSession.timestamp,
    state: workerState.simulationSession.state,
  });
}

export function applySimulationStop(session: SimulationSession) {
  session.state = {
    ...session.state,
    velocity: 0,
    steer: 0,
  };
  session.controlSequence = null;
  session.stopped = true;
  session.stateVersion += 1;
}

export function cloneControlSequence(controlSequence: LocalPlannerControlPoint[]): LocalPlannerControlPoint[] {
  return controlSequence.map((point) => ({
    timestamp: point.timestamp,
    targetVelocity: point.targetVelocity,
    targetSteer: point.targetSteer,
  }));
}

export function assertValidControlSequence(controlSequence: LocalPlannerControlPoint[]) {
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

export function setSimulationControlSequenceInternal(
  session: SimulationSession,
  controlSequence: LocalPlannerControlPoint[] | null,
) {
  if (session.stopped) {
    return;
  }

  if (!controlSequence || controlSequence.length === 0) {
    session.controlSequence = null;
    session.stateVersion += 1;
    return;
  }

  assertValidControlSequence(controlSequence);
  session.controlSequence = cloneControlSequence(controlSequence);
  session.stateVersion += 1;
}

export function sampleControl(controlSequence: LocalPlannerControlPoint[] | null, timestamp: number) {
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

    const prev = controlSequence[index - 1];
    const ratio = (timestamp - prev.timestamp) / (next.timestamp - prev.timestamp);
    return {
      timestamp,
      targetVelocity: prev.targetVelocity + (next.targetVelocity - prev.targetVelocity) * ratio,
      targetSteer: prev.targetSteer + (next.targetSteer - prev.targetSteer) * ratio,
    };
  }

  return last;
}

export function clearSimulationTimers() {
  if (!workerState.simulationSession) return;
  if (workerState.simulationSession.simulationTimerId !== null) {
    clearTimeout(workerState.simulationSession.simulationTimerId);
    workerState.simulationSession.simulationTimerId = null;
  }
  if (workerState.simulationSession.publishTimerId !== null) {
    clearInterval(workerState.simulationSession.publishTimerId);
    workerState.simulationSession.publishTimerId = null;
  }
}

export async function computeStepCarState(
  current: WasmCarState,
  targetVelocity: number,
  targetSteer: number,
  dt: number,
) {
  const config = await ensureWasmCore();
  const state = new CarState(current.x, current.y, current.yaw, current.velocity, current.steer);
  try {
    const next = state.stepped(config, targetVelocity, targetSteer, dt);
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

export async function computeOpenLoopStepCarState(current: WasmCarState, dt: number) {
  return computeStepCarState(current, current.velocity, current.steer, dt);
}

export async function advanceSimulation(session: SimulationSession) {
  const sampledControl = sampleControl(session.controlSequence, session.timestamp);
  if (!sampledControl) {
    return computeOpenLoopStepCarState(session.state, session.simDeltaTime);
  }
  return computeStepCarState(
    session.state,
    sampledControl.targetVelocity,
    sampledControl.targetSteer,
    session.simDeltaTime,
  );
}

export function scheduleSimulationTick(session: SimulationSession, simulationIntervalMs: number, loopToken: number) {
  session.simulationTimerId = setTimeout(() => {
    if (
      !workerState.simulationSession ||
      workerState.simulationSession !== session ||
      session.loopToken !== loopToken
    ) {
      return;
    }

    const stateVersion = session.stateVersion;
    session.timestamp += session.simDeltaTime;

    void advanceSimulation(session)
      .then((next) => {
        if (
          !workerState.simulationSession ||
          workerState.simulationSession !== session ||
          session.loopToken !== loopToken ||
          session.stateVersion !== stateVersion
        ) {
          return;
        }
        session.state = next;
      })
      .catch((error) => {
        console.error('Failed to advance simulation', error);
      })
      .finally(() => {
        if (
          !workerState.simulationSession ||
          workerState.simulationSession !== session ||
          session.loopToken !== loopToken
        ) {
          return;
        }
        scheduleSimulationTick(session, simulationIntervalMs, loopToken);
      });
  }, simulationIntervalMs);
}

export function startSimulationLoop(simulationIntervalMs: number, publishIntervalMs: number) {
  if (!workerState.simulationSession) return;
  clearSimulationTimers();

  workerState.simulationSession.loopToken += 1;
  const loopToken = workerState.simulationSession.loopToken;
  scheduleSimulationTick(workerState.simulationSession, simulationIntervalMs, loopToken);

  workerState.simulationSession.publishTimerId = setInterval(() => {
    emitSimulationState();
  }, publishIntervalMs);
}

export function ensureLocalPlannerSession() {
  if (!workerState.localPlannerSession) {
    workerState.localPlannerSession = {
      tracker: null,
      latestState: null,
      simDeltaTime: DEFAULT_LOCAL_PLANNER_DT,
      updateIntervalMs: DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS,
      updateTimerId: null,
      updateInFlight: false,
    };
  }

  if (workerState.localPlannerSession.updateTimerId === null) {
    workerState.localPlannerSession.updateTimerId = setInterval(() => {
      if (!workerState.localPlannerSession || workerState.localPlannerSession.updateInFlight) {
        return;
      }

      const tracker = workerState.localPlannerSession.tracker;
      const latestState = workerState.localPlannerSession.latestState;
      if (!tracker || !latestState) {
        return;
      }

      const activeSession = workerState.localPlannerSession;
      activeSession.updateInFlight = true;
      void runLocalPlannerUpdate(tracker, latestState.state, latestState.timestamp, activeSession.simDeltaTime)
        .then((result) => {
          if (!result || !workerState.localPlannerSession || workerState.localPlannerSession !== activeSession) {
            return;
          }

          if (workerState.simulationSession) {
            setSimulationControlSequenceInternal(workerState.simulationSession, result.controlSequence);
          }
          postEvent('localPlannerUpdate', result);
        })
        .catch((error) => {
          console.error('Failed to update local planner', error);
        })
        .finally(() => {
          if (workerState.localPlannerSession === activeSession) {
            activeSession.updateInFlight = false;
          }
        });
    }, workerState.localPlannerSession.updateIntervalMs);
  }

  return workerState.localPlannerSession;
}

export function clearLocalPlannerTimer() {
  if (!workerState.localPlannerSession || workerState.localPlannerSession.updateTimerId === null) {
    return;
  }
  clearInterval(workerState.localPlannerSession.updateTimerId);
  workerState.localPlannerSession.updateTimerId = null;
}

export { DEFAULT_PUBLISH_INTERVAL_MS, DEFAULT_SIM_DELTA_TIME, DEFAULT_SIM_INTERVAL_MS };

import type { Mode } from '../lib/appModel';
import type { HybridAStarStartSeedPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';
import type { AppRefs, AppStateUpdater, HistoryPoint } from './appRuntimeTypes';

export type PlanningControllerParams = {
  mode: Mode;
  refs: AppRefs;
  updateState: AppStateUpdater;
  replanMaxSpeed: number;
  toHybridAStarStartSeed: (points: LocalPlannerReferencePoint[]) => HybridAStarStartSeedPoint[];
};

export function hideGoalUnreachable(updateState: AppStateUpdater) {
  updateState('goalUnreachable', (current) => ({ ...current, visible: false }));
}

export const INITIAL_SIMULATION_TIMESTAMP = 0;

export function createInitialHistory(): HistoryPoint[] {
  return [{ t: INITIAL_SIMULATION_TIMESTAMP, value: 0 }];
}

export function resetPlanningInteractionState(refs: AppRefs, updateState: AppStateUpdater) {
  refs.dragStartRef.current = null;
  updateState('pressedPose', null);
  hideGoalUnreachable(updateState);
}

export function resetSimulationSessionState(refs: AppRefs, updateState: AppStateUpdater) {
  refs.dragStartRef.current = null;
  refs.goalRef.current = null;
  refs.globalTrajectoryRef.current = null;
  refs.localPlanningRef.current = false;
  refs.brakeTrajectoryRef.current = null;
  updateState('timestamp', INITIAL_SIMULATION_TIMESTAMP);
  updateState('goal', null);
  updateState('pressedPose', null);
  updateState('globalTrajectory', null);
  updateState('localTrajectory', []);
  updateState('referencePoints', []);
  updateState('globalPlannerSegments', []);
  updateState('velocityHistory', createInitialHistory());
  updateState('steerHistory', createInitialHistory());
  hideGoalUnreachable(updateState);
}

export function clearGoalPlanState(refs: AppRefs, updateState: AppStateUpdater, goalUnreachableVisible: boolean) {
  updateState('globalTrajectory', null);
  refs.globalTrajectoryRef.current = null;
  refs.localPlanningRef.current = false;
  updateState('goalUnreachable', (current) => ({ ...current, visible: goalUnreachableVisible }));
}

import type { Mode } from '../lib/appModel';
import type { HybridAStarStartSeedPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';
import type { AppRefs, AppStateUpdater } from './appRuntimeTypes';

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

export function resetPlanningInteractionState(refs: AppRefs, updateState: AppStateUpdater) {
  refs.dragStartRef.current = null;
  updateState('pressedPose', null);
  hideGoalUnreachable(updateState);
}

export function clearGoalPlanState(refs: AppRefs, updateState: AppStateUpdater, goalUnreachableVisible: boolean) {
  updateState('globalTrajectory', null);
  refs.globalTrajectoryRef.current = null;
  refs.localPlanningRef.current = false;
  updateState('goalUnreachable', (current) => ({ ...current, visible: goalUnreachableVisible }));
}

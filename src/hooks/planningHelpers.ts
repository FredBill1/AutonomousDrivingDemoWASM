import type React from 'react';

import type { Mode } from '../lib/appModel';
import type { AppRefs, AppSetters } from './appRuntimeTypes';
import type { HybridAStarStartSeedPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';

export type UsePlanningCallbacksParams = {
  mode: Mode;
  refs: AppRefs;
  setters: AppSetters;
  replanMaxSpeed: number;
  toHybridAStarStartSeed: (points: LocalPlannerReferencePoint[]) => HybridAStarStartSeedPoint[];
};

export function resetPlanningInteractionState(refs: AppRefs, setters: AppSetters) {
  refs.dragStartRef.current = null;
  setters.setPressedPose(null);
  setters.setGoalUnreachable((current) => ({ ...current, visible: false }));
}

export function clearGoalPlanState(
  refs: AppRefs,
  setters: AppSetters,
  goalUnreachable: { visible: boolean },
) {
  setters.setGlobalTrajectory(null);
  refs.globalTrajectoryRef.current = null;
  refs.localPlanningRef.current = false;
  setters.setGoalUnreachable((current) => ({ ...current, visible: goalUnreachable.visible }));
}

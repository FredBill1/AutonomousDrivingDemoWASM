import type React from 'react';

import type { CarState, Mode } from '../lib/appModel';
import type { DragStartState, GoalUnreachableState, MapServerSnapshot } from '../lib/appTypes';
import type { MapServerNode } from '../lib/mapServerNode';
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import type {
  HybridAStarProgress,
  HybridAStarStartSeedPoint,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerTrajectoryPoint,
} from '../lib/wasmCore';
import type { PlanningRefs, PlanningSetters } from './planningTypes';

export type UsePlanningCallbacksParams = PlanningRefs &
  PlanningSetters & {
  mode: Mode;
  goalRef: React.RefObject<CarState | null>;
  dragStartRef: React.RefObject<DragStartState | null>;
  globalTrajectoryRef: React.RefObject<LocalPlannerTrajectoryPoint[] | null>;
  setCar: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoal: React.Dispatch<React.SetStateAction<CarState | null>>;
  setPressedPose: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoalUnreachable: React.Dispatch<React.SetStateAction<GoalUnreachableState>>;
  setGlobalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerTrajectoryPoint[] | null>>;
  replanMaxSpeed: number;
  toHybridAStarStartSeed: (points: LocalPlannerReferencePoint[]) => HybridAStarStartSeedPoint[];
};

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

export type UsePlanningCallbacksParams = {
  mode: Mode;
  carRef: React.RefObject<CarState | null>;
  goalRef: React.RefObject<CarState | null>;
  mapSnapshotRef: React.RefObject<MapServerSnapshot>;
  globalTrajectoryRef: React.RefObject<LocalPlannerTrajectoryPoint[] | null>;
  brakeTrajectoryRef: React.RefObject<LocalPlannerReferencePoint[] | null>;
  dragStartRef: React.RefObject<DragStartState | null>;
  planningRequestRef: React.RefObject<number>;
  localPlanningRef: React.RefObject<boolean>;
  trajectoryCollisionCheckingNodeRef: React.RefObject<TrajectoryCollisionCheckingNode | null>;
  mapServerNodeRef: React.RefObject<MapServerNode | null>;
  setCar: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoal: React.Dispatch<React.SetStateAction<CarState | null>>;
  setPressedPose: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoalUnreachable: React.Dispatch<React.SetStateAction<GoalUnreachableState>>;
  setGlobalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerTrajectoryPoint[] | null>>;
  setGlobalPlannerSegments: React.Dispatch<React.SetStateAction<HybridAStarProgress['segments'][]>>;
  setLocalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerPathPoint[]>>;
  setReferencePoints: React.Dispatch<React.SetStateAction<LocalPlannerReferencePoint[]>>;
  setMapSnapshot: React.Dispatch<React.SetStateAction<MapServerSnapshot>>;
  replanMaxSpeed: number;
  toHybridAStarStartSeed: (points: LocalPlannerReferencePoint[]) => HybridAStarStartSeedPoint[];
};

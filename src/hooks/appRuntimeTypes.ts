import type React from 'react';

import type { CarState, Mode } from '../lib/appModel';
import type { CarShape, DragStartState, GoalUnreachableState, MapServerSnapshot, MotionLimits } from '../lib/appTypes';
import type { MapServerNode } from '../lib/mapServerNode';
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import type {
  HybridAStarProgress,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerTrajectoryPoint,
} from '../lib/wasmCore';

export type HistoryPoint = {
  t: number;
  value: number;
};

export type AppRefs = {
  mapServerNodeRef: React.RefObject<MapServerNode | null>;
  carRef: React.RefObject<CarState | null>;
  timestampRef: React.RefObject<number>;
  goalRef: React.RefObject<CarState | null>;
  mapSnapshotRef: React.RefObject<MapServerSnapshot>;
  globalTrajectoryRef: React.RefObject<LocalPlannerTrajectoryPoint[] | null>;
  localPlanningRef: React.RefObject<boolean>;
  brakeTrajectoryRef: React.RefObject<LocalPlannerReferencePoint[] | null>;
  planningRequestRef: React.RefObject<number>;
  dragStartRef: React.RefObject<DragStartState | null>;
  trajectoryCollisionCheckingNodeRef: React.RefObject<TrajectoryCollisionCheckingNode | null>;
};

export type AppSetters = {
  setMode: React.Dispatch<React.SetStateAction<Mode>>;
  setTimestamp: React.Dispatch<React.SetStateAction<number>>;
  setMapSnapshot: React.Dispatch<React.SetStateAction<MapServerSnapshot>>;
  setCarShape: React.Dispatch<React.SetStateAction<CarShape | null>>;
  setMotionLimits: React.Dispatch<React.SetStateAction<MotionLimits | null>>;
  setCar: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoal: React.Dispatch<React.SetStateAction<CarState | null>>;
  setPressedPose: React.Dispatch<React.SetStateAction<CarState | null>>;
  setGoalUnreachable: React.Dispatch<React.SetStateAction<GoalUnreachableState>>;
  setGlobalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerTrajectoryPoint[] | null>>;
  setLocalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerPathPoint[]>>;
  setReferencePoints: React.Dispatch<React.SetStateAction<LocalPlannerReferencePoint[]>>;
  setGlobalPlannerSegments: React.Dispatch<React.SetStateAction<HybridAStarProgress['segments'][]>>;
  setVelocityHistory: React.Dispatch<React.SetStateAction<HistoryPoint[]>>;
  setSteerHistory: React.Dispatch<React.SetStateAction<HistoryPoint[]>>;
};

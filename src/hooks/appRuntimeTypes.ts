import type React from 'react';

import type { CarState, Mode } from '../lib/appModel';
import type { CarShape, DragStartState, GoalUnreachableState, MapServerSnapshot, MotionLimits } from '../lib/appTypes';
import type { MapServerNode } from '../lib/mapServerNode';
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import type {
  HybridAStarProgressSegment,
  LocalPlannerPathPoint,
  LocalPlannerReferencePoint,
  LocalPlannerTrajectoryPoint,
} from '../lib/wasmCore';

export type HistoryPoint = {
  t: number;
  value: number;
};

export type AppState = {
  mode: Mode;
  timestamp: number;
  mapSnapshot: MapServerSnapshot;
  carShape: CarShape | null;
  motionLimits: MotionLimits | null;
  car: CarState | null;
  goal: CarState | null;
  pressedPose: CarState | null;
  goalUnreachable: GoalUnreachableState;
  globalTrajectory: LocalPlannerTrajectoryPoint[] | null;
  localTrajectory: LocalPlannerPathPoint[];
  referencePoints: LocalPlannerReferencePoint[];
  globalPlannerSegments: HybridAStarProgressSegment[][];
  velocityHistory: HistoryPoint[];
  steerHistory: HistoryPoint[];
};

export type StateUpdater<T> = T | ((current: T) => T);

export type AppStateUpdater = <Key extends keyof AppState>(key: Key, updater: StateUpdater<AppState[Key]>) => void;

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

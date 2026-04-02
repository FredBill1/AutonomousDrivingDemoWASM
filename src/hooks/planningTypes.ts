import type React from 'react';

import type { CarState } from '../lib/appModel';
import type { MapServerSnapshot } from '../lib/appTypes';
import type { MapServerNode } from '../lib/mapServerNode';
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import type { HybridAStarProgress, LocalPlannerPathPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';

export type PlanningRefs = {
  planningRequestRef: React.RefObject<number>;
  mapServerNodeRef: React.RefObject<MapServerNode | null>;
  trajectoryCollisionCheckingNodeRef: React.RefObject<TrajectoryCollisionCheckingNode | null>;
  carRef: React.RefObject<CarState | null>;
  localPlanningRef: React.RefObject<boolean>;
  brakeTrajectoryRef: React.RefObject<LocalPlannerReferencePoint[] | null>;
  mapSnapshotRef: React.RefObject<MapServerSnapshot>;
};

export type PlanningSetters = {
  setGlobalPlannerSegments: React.Dispatch<React.SetStateAction<HybridAStarProgress['segments'][]>>;
  setLocalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerPathPoint[]>>;
  setReferencePoints: React.Dispatch<React.SetStateAction<LocalPlannerReferencePoint[]>>;
  setMapSnapshot: React.Dispatch<React.SetStateAction<MapServerSnapshot>>;
};

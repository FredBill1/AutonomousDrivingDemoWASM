import type { CarState, Mode, Obstacle } from '../lib/appModel';
import type { CarShape, GoalUnreachableState, MotionLimits } from '../lib/appTypes';
import type { MapBoundingBox } from '../lib/mapServerNode';
import type { PathPoint } from '../lib/mapViewportDraw';
import type { HybridAStarProgress, LocalPlannerPathPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';

export type MapViewportProps = {
  bounds: MapBoundingBox;
  mode: Mode;
  carShape: CarShape | null;
  motionLimits: MotionLimits | null;
  knownObstacles: Obstacle[];
  unknownObstacles: Obstacle[];
  car: CarState | null;
  goal: CarState | null;
  pressedPose: CarState | null;
  goalUnreachable: GoalUnreachableState;
  globalTrajectory: PathPoint[] | null;
  localTrajectory: LocalPlannerPathPoint[];
  referencePoints: LocalPlannerReferencePoint[];
  globalPlannerSegments: HybridAStarProgress['segments'][];
  onPrimaryDragStart: (world: { x: number; y: number }) => boolean;
  onPrimaryDragMove: (world: { x: number; y: number }) => void;
  onPrimaryDragEnd: (world: { x: number; y: number }) => void;
  onPrimaryDragCancel: () => void;
};

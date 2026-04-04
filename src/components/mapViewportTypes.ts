import type { ViewportConfig } from '../lib/appConfig';
import type { CarState, Mode, Obstacle } from '../lib/appModel';
import type { CarShape, GoalUnreachableState, MotionLimits } from '../lib/appTypes';
import type { MapBoundingBox } from '../lib/mapServerNode';
import type { PathPoint } from '../lib/mapViewportDraw';
import type { HybridAStarProgressSegment, LocalPlannerPathPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';

export type MapViewportScene = {
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
  globalPlannerSegments: HybridAStarProgressSegment[][];
};

export type MapViewportInteractionHandlers = {
  onPrimaryDragStart: (world: { x: number; y: number }) => boolean;
  onPrimaryDragMove: (world: { x: number; y: number }) => void;
  onPrimaryDragEnd: (world: { x: number; y: number }) => void;
  onPrimaryDragCancel: () => void;
};

export type MapViewportProps = {
  scene: MapViewportScene;
  interaction: MapViewportInteractionHandlers;
  viewportConfig: ViewportConfig;
};

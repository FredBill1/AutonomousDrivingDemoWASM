import { CAR_CONSTANTS } from './appModel';
import type { MapBoundingBox } from './mapServerNode';

export type { MapServerSnapshot } from './mapServerNode';
export type { MapBoundingBox } from './mapServerNode';

export type CarShape = {
    wheelBase: number;
    length: number;
    width: number;
    backToWheel: number;
    wheelLength: number;
    wheelWidth: number;
    wheelSpacing: number;
    backToCenter: number;
};

export type MotionLimits = {
    scanRadius: number;
    maxSpeedKmh: number;
    minSpeedKmh: number;
    maxSteerDeg: number;
};

export type GoalUnreachableState = {
    visible: boolean;
    x: number;
    y: number;
};

export type DragStartState = {
    startX: number;
    startY: number;
};

export type DashboardLayout = 'split' | 'stacked';

export const FALLBACK_MAP_BOUNDING_BOX: MapBoundingBox = {
    minX: 0,
    minY: 0,
    maxX: 80,
    maxY: 60,
};

export const DEFAULT_CAR_SHAPE: CarShape = {
    wheelBase: CAR_CONSTANTS.wheelBase,
    length: CAR_CONSTANTS.length,
    width: CAR_CONSTANTS.width,
    backToWheel: CAR_CONSTANTS.backToWheel,
    wheelLength: CAR_CONSTANTS.wheelLength,
    wheelWidth: CAR_CONSTANTS.wheelWidth,
    wheelSpacing: CAR_CONSTANTS.wheelSpacing,
    backToCenter: CAR_CONSTANTS.backToCenter,
};

export const DEFAULT_MOTION_LIMITS: MotionLimits = {
    scanRadius: CAR_CONSTANTS.scanRadius,
    maxSpeedKmh: CAR_CONSTANTS.maxSpeed * 3.6,
    minSpeedKmh: CAR_CONSTANTS.minSpeed * 3.6,
    maxSteerDeg: (CAR_CONSTANTS.maxSteer * 180) / Math.PI,
};

export const STACKED_LAYOUT_MAX_WIDTH_PX = 560;
export const STACKED_LAYOUT_MIN_MAP_HEIGHT_PX = 220;
export const STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX = 110;
export const STACKED_LAYOUT_GAP_PX = 8;

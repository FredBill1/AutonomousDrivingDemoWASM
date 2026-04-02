import type { Viewport } from 'pixi-viewport';
import type { Graphics, Text } from 'pixi.js';
import type React from 'react';

import type { CarState, Mode, Obstacle, PathPoint } from './appModel';
import type { CarShape, GoalUnreachableState, MotionLimits } from './appTypes';
import type { MapBoundingBox } from './mapServerNode';
import type { HybridAStarProgress, LocalPlannerPathPoint, LocalPlannerReferencePoint } from './wasmCore';

export type { PathPoint } from './appModel';

export type DrawLayers = {
  grid: Graphics;
  boundary: Graphics;
  segments: Graphics;
  unknownObstacles: Graphics;
  knownObstacles: Graphics;
  globalTrajectory: Graphics;
  localTrajectory: Graphics;
  referencePoints: Graphics;
  scanRing: Graphics;
  cars: Graphics;
  label: Text;
};

export type DrawParams = {
  bounds: MapBoundingBox;
  globalPlannerSegments: HybridAStarProgress['segments'][];
  unknownObstacles: Obstacle[];
  knownObstacles: Obstacle[];
  globalTrajectory: PathPoint[] | null;
  localTrajectory: LocalPlannerPathPoint[];
  referencePoints: LocalPlannerReferencePoint[];
  car: CarState | null;
  carShape: CarShape | null;
  goal: CarState | null;
  motionLimits: MotionLimits | null;
  pressedPose: CarState | null;
  mode: Mode;
  goalUnreachable: GoalUnreachableState;
};

export const GRID_SPACING = 5;

export function clamp(value: number, minValue: number, maxValue: number) {
  return Math.min(maxValue, Math.max(minValue, value));
}

export { syncCanvasElementSize } from './pixiAppInit';

export function worldWidth(bounds: MapBoundingBox) {
  return bounds.maxX - bounds.minX;
}

export function worldHeight(bounds: MapBoundingBox) {
  return bounds.maxY - bounds.minY;
}

export function toViewportX(x: number, bounds: MapBoundingBox) {
  return x - bounds.minX;
}

export function toViewportY(y: number, bounds: MapBoundingBox) {
  return bounds.maxY - y;
}

export function toViewportPoint(point: { x: number; y: number }, bounds: MapBoundingBox) {
  return {
    x: toViewportX(point.x, bounds),
    y: toViewportY(point.y, bounds),
  };
}

export function drawPolyline(
  graphics: Graphics,
  points: PathPoint[],
  bounds: MapBoundingBox,
  width: number,
  color: number,
  alpha: number,
) {
  if (points.length < 2) {
    return;
  }

  points.forEach((point, index) => {
    const viewPoint = toViewportPoint(point, bounds);
    if (index === 0) {
      graphics.moveTo(viewPoint.x, viewPoint.y);
    } else {
      graphics.lineTo(viewPoint.x, viewPoint.y);
    }
  });
  graphics.stroke({ width, color, alpha, cap: 'round', join: 'round' });
}

export function transformCarPoint(car: Pick<CarState, 'x' | 'y' | 'yaw'>, localX: number, localY: number) {
  return {
    x: car.x + localX * Math.cos(car.yaw) - localY * Math.sin(car.yaw),
    y: car.y + localX * Math.sin(car.yaw) + localY * Math.cos(car.yaw),
  };
}

export function buildCarPolygon(car: Pick<CarState, 'x' | 'y' | 'yaw'>, shape: CarShape) {
  const local = [
    [-shape.backToWheel, -shape.width / 2],
    [shape.length - shape.backToWheel, -shape.width / 2],
    [shape.length - shape.backToWheel, shape.width / 2],
    [-shape.backToWheel, shape.width / 2],
  ];

  return local.flatMap(([localX, localY]) => {
    const point = transformCarPoint(car, localX, localY);
    return [point.x, point.y];
  });
}

export function buildWheelPolygons(car: Pick<CarState, 'x' | 'y' | 'yaw' | 'steer'>, shape: CarShape) {
  const halfWheelLength = shape.wheelLength / 2;
  const halfWheelWidth = shape.wheelWidth / 2;
  const wheelBox = [
    [-halfWheelLength, -halfWheelWidth],
    [halfWheelLength, -halfWheelWidth],
    [halfWheelLength, halfWheelWidth],
    [-halfWheelLength, halfWheelWidth],
  ];
  const cosSteer = Math.cos(car.steer);
  const sinSteer = Math.sin(car.steer);
  const rotateFrontWheelPoint = ([x, y]: number[]) => [x * cosSteer - y * sinSteer, x * sinSteer + y * cosSteer];

  const frontWheel = wheelBox.map(rotateFrontWheelPoint);
  const frontLeftWheel = frontWheel.map(([x, y]) => [x + shape.wheelBase, y + shape.wheelSpacing / 2]);
  const frontRightWheel = frontWheel.map(([x, y]) => [x + shape.wheelBase, y - shape.wheelSpacing / 2]);
  const rearLeftWheel = wheelBox.map(([x, y]) => [x, y + shape.wheelSpacing / 2]);
  const rearRightWheel = wheelBox.map(([x, y]) => [x, y - shape.wheelSpacing / 2]);

  return [frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel].map((wheel) =>
    wheel.flatMap(([localX, localY]) => {
      const point = transformCarPoint(car, localX, localY);
      return [point.x, point.y];
    }),
  );
}

export function drawCar(
  graphics: Graphics,
  car: Pick<CarState, 'x' | 'y' | 'yaw' | 'steer'>,
  bounds: MapBoundingBox,
  shape: CarShape,
  strokeColor: number,
) {
  const bodyPolygon = buildCarPolygon(car, shape);
  const bodyPoints = bodyPolygon.flatMap((value, index) =>
    index % 2 === 0 ? [toViewportX(value, bounds)] : [toViewportY(value, bounds)],
  );
  graphics
    .poly(bodyPoints, true)
    .fill({ color: 0xffffff, alpha: 0.08 })
    .stroke({ width: 0.2, color: strokeColor, alpha: 1 });

  const wheelPolygons = buildWheelPolygons(car, shape);
  wheelPolygons.forEach((wheelPolygon) => {
    const points = wheelPolygon.flatMap((value, index) =>
      index % 2 === 0 ? [toViewportX(value, bounds)] : [toViewportY(value, bounds)],
    );
    graphics.poly(points, true).stroke({ width: 0.15, color: strokeColor, alpha: 1 });
  });

  const rearAxle = toViewportPoint({ x: car.x, y: car.y }, bounds);
  graphics
    .circle(rearAxle.x, rearAxle.y, 0.12)
    .fill({ color: strokeColor, alpha: 1 })
    .stroke({ width: 0.25, color: strokeColor, alpha: 1 });
}

export function setViewportTransform(
  viewport: Viewport,
  screenX: number,
  screenY: number,
  scale: number,
  worldPoint: { x: number; y: number },
) {
  viewport.setZoom(scale);
  viewport.position.set(screenX - worldPoint.x * viewport.scale.x, screenY - worldPoint.y * viewport.scale.y);
}

export function performDraw(layers: DrawLayers, viewportRef: React.RefObject<Viewport | null>, params: DrawParams) {
  const {
    bounds,
    globalPlannerSegments,
    unknownObstacles,
    knownObstacles,
    globalTrajectory,
    localTrajectory,
    referencePoints,
    car,
    carShape,
    goal,
    motionLimits,
    pressedPose,
    mode,
    goalUnreachable,
  } = params;

  layers.grid.clear();
  const width = worldWidth(bounds);
  const height = worldHeight(bounds);
  for (let x = 0; x <= width; x += GRID_SPACING) {
    layers.grid.moveTo(x, 0);
    layers.grid.lineTo(x, height);
  }
  for (let y = 0; y <= height; y += GRID_SPACING) {
    layers.grid.moveTo(0, y);
    layers.grid.lineTo(width, y);
  }
  layers.grid.stroke({ width: 0.1, color: 0xffffff, alpha: 0.08 });

  layers.boundary.clear();
  layers.boundary.rect(0, 0, width, height).stroke({ width: 0.3, color: 0xffffff, alpha: 0.18 });

  layers.segments.clear();
  globalPlannerSegments.forEach((segmentGroup) => {
    segmentGroup.forEach((segment) => {
      layers.segments.moveTo(toViewportX(segment.x1, bounds), toViewportY(segment.y1, bounds));
      layers.segments.lineTo(toViewportX(segment.x2, bounds), toViewportY(segment.y2, bounds));
    });
  });
  layers.segments.stroke({ width: 0.1, color: 0x57d8ff, alpha: 0.18 });

  layers.unknownObstacles.clear();
  unknownObstacles.forEach((obstacle) => {
    const point = toViewportPoint(obstacle, bounds);
    layers.unknownObstacles.circle(point.x, point.y, 0.28).fill({ color: 0x57d8ff, alpha: 0.95 });
  });

  layers.knownObstacles.clear();
  knownObstacles.forEach((obstacle) => {
    const point = toViewportPoint(obstacle, bounds);
    layers.knownObstacles.circle(point.x, point.y, 0.24).fill({ color: 0xff6f6f, alpha: 0.95 });
  });

  layers.globalTrajectory.clear();
  if (globalTrajectory) {
    drawPolyline(layers.globalTrajectory, globalTrajectory, bounds, 0.3, 0x57d8ff, 0.95);
  }

  layers.localTrajectory.clear();
  drawPolyline(layers.localTrajectory, localTrajectory, bounds, 0.35, 0x9fe870, 0.96);

  layers.referencePoints.clear();
  referencePoints.forEach((point) => {
    const viewPoint = toViewportPoint(point, bounds);
    layers.referencePoints.moveTo(viewPoint.x - 0.35, viewPoint.y - 0.35);
    layers.referencePoints.lineTo(viewPoint.x + 0.35, viewPoint.y + 0.35);
    layers.referencePoints.moveTo(viewPoint.x - 0.35, viewPoint.y + 0.35);
    layers.referencePoints.lineTo(viewPoint.x + 0.35, viewPoint.y - 0.35);
  });
  layers.referencePoints.stroke({ width: 0.25, color: 0xff7a7a, alpha: 0.94 });

  layers.scanRing.clear();
  if (car && carShape && motionLimits) {
    const scanCenter = toViewportPoint(
      {
        x: car.x + carShape.backToCenter * Math.cos(car.yaw),
        y: car.y + carShape.backToCenter * Math.sin(car.yaw),
      },
      bounds,
    );
    layers.scanRing
      .circle(scanCenter.x, scanCenter.y, motionLimits.scanRadius)
      .fill({ color: 0x57d8ff, alpha: 0.05 })
      .stroke({ width: 0.24, color: 0x57d8ff, alpha: 0.32 });
  }

  layers.cars.clear();
  if (carShape) {
    if (car) {
      drawCar(layers.cars, car, bounds, carShape, 0xffffff);
    }
    if (goal) {
      drawCar(layers.cars, goal, bounds, carShape, 0x9fe870);
    }
    if (pressedPose) {
      drawCar(layers.cars, pressedPose, bounds, carShape, mode === 'pose' ? 0xffffff : 0x9fe870);
    }
  }

  layers.label.visible = goalUnreachable.visible;
  if (goalUnreachable.visible) {
    const labelPoint = toViewportPoint(goalUnreachable, bounds);
    const viewport = viewportRef.current;
    if (!viewport) {
      return;
    }
    const screenPoint = viewport.toScreen(labelPoint);
    layers.label.position.set(screenPoint.x, screenPoint.y);
  }
}

import { useCallback, useEffect, useRef } from 'react';

import { Viewport } from 'pixi-viewport';
import { type Application, Graphics, Text } from 'pixi.js';

import { usePixiLifecycle } from '../hooks/usePixiLifecycle';
import type { CarState, Mode, Obstacle } from '../lib/appModel';
import type { CarShape, GoalUnreachableState, MotionLimits } from '../lib/appTypes';
import type { MapBoundingBox } from '../lib/mapServerNode';
import { type DrawLayers, type PathPoint, performDraw, worldHeight, worldWidth } from '../lib/mapViewportDraw';
import { type TouchState, createPointerHandlers } from '../lib/mapViewportInteraction';
import type { HybridAStarProgress, LocalPlannerPathPoint, LocalPlannerReferencePoint } from '../lib/wasmCore';

type MapViewportProps = {
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

export function MapViewport({
  bounds,
  mode,
  carShape,
  motionLimits,
  knownObstacles,
  unknownObstacles,
  car,
  goal,
  pressedPose,
  goalUnreachable,
  globalTrajectory,
  localTrajectory,
  referencePoints,
  globalPlannerSegments,
  onPrimaryDragStart,
  onPrimaryDragMove,
  onPrimaryDragEnd,
  onPrimaryDragCancel,
}: MapViewportProps) {
  const hostRef = useRef<HTMLDivElement | null>(null);
  const appRef = useRef<Application | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const viewportRef = useRef<Viewport | null>(null);
  const layersRef = useRef<DrawLayers | null>(null);
  const boundsRef = useRef(bounds);
  const onPrimaryDragStartRef = useRef(onPrimaryDragStart);
  const onPrimaryDragMoveRef = useRef(onPrimaryDragMove);
  const onPrimaryDragEndRef = useRef(onPrimaryDragEnd);
  const onPrimaryDragCancelRef = useRef(onPrimaryDragCancel);
  const middlePanRef = useRef<{ pointerId: number; lastX: number; lastY: number } | null>(null);
  const primaryDragRef = useRef<{ pointerId: number; pointerType: string } | null>(null);
  const touchStateRef = useRef<TouchState>({ points: new Map(), gesture: null });
  const fittedBoundsKeyRef = useRef<string | null>(null);
  const fitScaleRef = useRef(1);
  const drawRef = useRef<() => void>(() => {});

  useEffect(() => {
    boundsRef.current = bounds;
  }, [bounds]);

  useEffect(() => {
    onPrimaryDragStartRef.current = onPrimaryDragStart;
    onPrimaryDragMoveRef.current = onPrimaryDragMove;
    onPrimaryDragEndRef.current = onPrimaryDragEnd;
    onPrimaryDragCancelRef.current = onPrimaryDragCancel;
  }, [onPrimaryDragCancel, onPrimaryDragEnd, onPrimaryDragMove, onPrimaryDragStart]);

  const onPixiReady = useCallback(({ app }: { app: Application }) => {
    const { handlePointerDown, handlePointerMove, finishPointer, handleWheel, handleContextMenu } =
      createPointerHandlers({
        viewportRef,
        boundsRef,
        hostRef,
        primaryDragRef,
        middlePanRef,
        touchStateRef,
        fitScaleRef,
        canvasRef,
        onPrimaryDragStartRef,
        onPrimaryDragMoveRef,
        onPrimaryDragEndRef,
        onPrimaryDragCancelRef,
      });

    const viewport = new Viewport({
      screenWidth: app.renderer.width,
      screenHeight: app.renderer.height,
      worldWidth: worldWidth(boundsRef.current),
      worldHeight: worldHeight(boundsRef.current),
      events: app.renderer.events,
    });
    app.stage.addChild(viewport);

    const grid = new Graphics();
    const boundary = new Graphics();
    const segments = new Graphics();
    const unknownObstaclesLayer = new Graphics();
    const knownObstaclesLayer = new Graphics();
    const globalTrajectoryLayer = new Graphics();
    const localTrajectoryLayer = new Graphics();
    const referencePointsLayer = new Graphics();
    const scanRingLayer = new Graphics();
    const carsLayer = new Graphics();
    const label = new Text({
      text: 'Goal is unreachable',
      style: {
        fill: 0xff7b7b,
        fontFamily: 'Bahnschrift, Trebuchet MS, Segoe UI, sans-serif',
        fontSize: 18,
        fontWeight: '700',
        align: 'center',
      },
    });
    label.anchor.set(0.5);

    viewport.addChild(grid);
    viewport.addChild(boundary);
    viewport.addChild(segments);
    viewport.addChild(unknownObstaclesLayer);
    viewport.addChild(knownObstaclesLayer);
    viewport.addChild(globalTrajectoryLayer);
    viewport.addChild(localTrajectoryLayer);
    viewport.addChild(referencePointsLayer);
    viewport.addChild(scanRingLayer);
    viewport.addChild(carsLayer);
    app.stage.addChild(label);

    const handlePointerUp = (event: PointerEvent) => finishPointer(event, false);
    const handlePointerCancel = (event: PointerEvent) => finishPointer(event, true);
    app.canvas.addEventListener('pointerdown', handlePointerDown);
    app.canvas.addEventListener('pointermove', handlePointerMove);
    app.canvas.addEventListener('pointerup', handlePointerUp);
    app.canvas.addEventListener('pointercancel', handlePointerCancel);
    app.canvas.addEventListener('wheel', handleWheel, { passive: false });
    app.canvas.addEventListener('contextmenu', handleContextMenu);

    appRef.current = app;
    viewportRef.current = viewport;
    layersRef.current = {
      grid,
      boundary,
      segments,
      unknownObstacles: unknownObstaclesLayer,
      knownObstacles: knownObstaclesLayer,
      globalTrajectory: globalTrajectoryLayer,
      localTrajectory: localTrajectoryLayer,
      referencePoints: referencePointsLayer,
      scanRing: scanRingLayer,
      cars: carsLayer,
      label,
    };
    canvasRef.current = app.canvas;

    return {
      handleResize: () => {
        const currentBounds = boundsRef.current;
        viewport.resize(app.renderer.width, app.renderer.height, worldWidth(currentBounds), worldHeight(currentBounds));

        if (fittedBoundsKeyRef.current === null) {
          const scale = Math.min(
            app.renderer.width / Math.max(worldWidth(currentBounds), 1),
            app.renderer.height / Math.max(worldHeight(currentBounds), 1),
          );
          fitScaleRef.current = scale;
          viewport.setZoom(scale);
          viewport.position.set(
            (app.renderer.width - worldWidth(currentBounds) * scale) / 2,
            (app.renderer.height - worldHeight(currentBounds) * scale) / 2,
          );
          fittedBoundsKeyRef.current = `${currentBounds.minX}:${currentBounds.minY}:${currentBounds.maxX}:${currentBounds.maxY}`;
        }

        drawRef.current();
      },
      cleanup: () => {
        app.canvas.removeEventListener('pointerdown', handlePointerDown);
        app.canvas.removeEventListener('pointermove', handlePointerMove);
        app.canvas.removeEventListener('pointerup', handlePointerUp);
        app.canvas.removeEventListener('pointercancel', handlePointerCancel);
        app.canvas.removeEventListener('wheel', handleWheel);
        app.canvas.removeEventListener('contextmenu', handleContextMenu);
        middlePanRef.current = null;
        primaryDragRef.current = null;
        touchStateRef.current = { points: new Map(), gesture: null };
        canvasRef.current = null;
        layersRef.current = null;
        viewportRef.current?.destroy({ children: true });
        viewportRef.current = null;
        appRef.current = null;
      },
    };
  }, []);

  usePixiLifecycle(hostRef, onPixiReady);

  useEffect(() => {
    const viewport = viewportRef.current;
    if (!viewport) {
      return;
    }

    const key = `${bounds.minX}:${bounds.minY}:${bounds.maxX}:${bounds.maxY}`;
    const width = worldWidth(bounds);
    const height = worldHeight(bounds);
    viewport.resize(viewport.screenWidth, viewport.screenHeight, width, height);
    if (fittedBoundsKeyRef.current !== key) {
      const scale = Math.min(viewport.screenWidth / Math.max(width, 1), viewport.screenHeight / Math.max(height, 1));
      fitScaleRef.current = scale;
      viewport.setZoom(scale);
      viewport.position.set((viewport.screenWidth - width * scale) / 2, (viewport.screenHeight - height * scale) / 2);
      fittedBoundsKeyRef.current = key;
    }
  }, [bounds]);

  useEffect(() => {
    drawRef.current = () => {
      const layers = layersRef.current;
      if (!layers) {
        return;
      }
      performDraw(layers, viewportRef, {
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
      });
    };

    drawRef.current();
  }, [
    bounds,
    car,
    carShape,
    globalPlannerSegments,
    globalTrajectory,
    goal,
    goalUnreachable,
    knownObstacles,
    localTrajectory,
    mode,
    motionLimits,
    pressedPose,
    referencePoints,
    unknownObstacles,
  ]);

  return <div ref={hostRef} className="map-view" />;
}

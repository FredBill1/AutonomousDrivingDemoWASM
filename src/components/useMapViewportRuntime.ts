import { useEffect, useRef } from 'react';

import { Viewport } from 'pixi-viewport';
import { Application, Graphics, Text } from 'pixi.js';

import { type DrawLayers, performDraw, syncCanvasElementSize, worldHeight, worldWidth } from '../lib/mapViewportDraw';
import { type TouchState, createPointerHandlers } from '../lib/mapViewportInteraction';
import { setupPixiCanvas, setupResizeListeners } from '../lib/pixiAppInit';
import type { MapViewportProps } from './mapViewportTypes';

function useLatestCallbacks({
  onPrimaryDragStart,
  onPrimaryDragMove,
  onPrimaryDragEnd,
  onPrimaryDragCancel,
}: Pick<MapViewportProps, 'onPrimaryDragStart' | 'onPrimaryDragMove' | 'onPrimaryDragEnd' | 'onPrimaryDragCancel'>) {
  const onPrimaryDragStartRef = useRef(onPrimaryDragStart);
  const onPrimaryDragMoveRef = useRef(onPrimaryDragMove);
  const onPrimaryDragEndRef = useRef(onPrimaryDragEnd);
  const onPrimaryDragCancelRef = useRef(onPrimaryDragCancel);

  useEffect(() => {
    onPrimaryDragStartRef.current = onPrimaryDragStart;
    onPrimaryDragMoveRef.current = onPrimaryDragMove;
    onPrimaryDragEndRef.current = onPrimaryDragEnd;
    onPrimaryDragCancelRef.current = onPrimaryDragCancel;
  }, [onPrimaryDragCancel, onPrimaryDragEnd, onPrimaryDragMove, onPrimaryDragStart]);

  return {
    onPrimaryDragStartRef,
    onPrimaryDragMoveRef,
    onPrimaryDragEndRef,
    onPrimaryDragCancelRef,
  };
}

function createDrawLayers(viewport: Viewport, app: Application) {
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

  return {
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
  } satisfies DrawLayers;
}

function fitViewportToBounds(viewport: Viewport, bounds: MapViewportProps['bounds'], fitScaleRef: { current: number }) {
  const width = worldWidth(bounds);
  const height = worldHeight(bounds);
  const scale = Math.min(viewport.screenWidth / Math.max(width, 1), viewport.screenHeight / Math.max(height, 1));
  fitScaleRef.current = scale;
  viewport.setZoom(scale);
  viewport.position.set((viewport.screenWidth - width * scale) / 2, (viewport.screenHeight - height * scale) / 2);
}

export function useMapViewportRuntime({
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
  const { onPrimaryDragStartRef, onPrimaryDragMoveRef, onPrimaryDragEndRef, onPrimaryDragCancelRef } =
    useLatestCallbacks({
      onPrimaryDragStart,
      onPrimaryDragMove,
      onPrimaryDragEnd,
      onPrimaryDragCancel,
    });
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
    let disposed = false;
    let handlePointerUp: ((event: PointerEvent) => void) | null = null;
    let handlePointerCancel: ((event: PointerEvent) => void) | null = null;
    let initialResizeFrame = 0;
    const host = hostRef.current;
    if (!host) {
      return;
    }

    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    const app = new Application();
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

    const syncViewportSize = () => {
      const currentApp = appRef.current;
      const viewport = viewportRef.current;
      const currentHost = hostRef.current;
      if (!currentApp || !viewport || !currentHost) {
        return;
      }

      const nextWidth = Math.max(1, currentHost.clientWidth);
      const nextHeight = Math.max(1, currentHost.clientHeight);
      const currentBounds = boundsRef.current;
      currentApp.renderer.resize(nextWidth, nextHeight);
      syncCanvasElementSize(currentApp.canvas, nextWidth, nextHeight);
      viewport.resize(nextWidth, nextHeight, worldWidth(currentBounds), worldHeight(currentBounds));

      if (fittedBoundsKeyRef.current === null) {
        fitViewportToBounds(viewport, currentBounds, fitScaleRef);
        fittedBoundsKeyRef.current = `${currentBounds.minX}:${currentBounds.minY}:${currentBounds.maxX}:${currentBounds.maxY}`;
      }

      drawRef.current();
    };

    void app
      .init({
        width,
        height,
        antialias: true,
        autoDensity: true,
        backgroundAlpha: 0,
        preference: 'webgl',
        resolution: Math.min(window.devicePixelRatio || 1, 2),
      })
      .then(() => {
        if (!setupPixiCanvas(app, host, width, height, disposed)) {
          return;
        }

        canvasRef.current = app.canvas;

        const viewport = new Viewport({
          screenWidth: width,
          screenHeight: height,
          worldWidth: worldWidth(boundsRef.current),
          worldHeight: worldHeight(boundsRef.current),
          events: app.renderer.events,
        });

        app.stage.addChild(viewport);
        appRef.current = app;
        viewportRef.current = viewport;
        layersRef.current = createDrawLayers(viewport, app);

        handlePointerUp = (event: PointerEvent) => finishPointer(event, false);
        handlePointerCancel = (event: PointerEvent) => finishPointer(event, true);

        app.canvas.addEventListener('pointerdown', handlePointerDown);
        app.canvas.addEventListener('pointermove', handlePointerMove);
        app.canvas.addEventListener('pointerup', handlePointerUp);
        app.canvas.addEventListener('pointercancel', handlePointerCancel);
        app.canvas.addEventListener('wheel', handleWheel, { passive: false });
        app.canvas.addEventListener('contextmenu', handleContextMenu);

        syncViewportSize();
        initialResizeFrame = window.requestAnimationFrame(syncViewportSize);
      });

    const removeResizeListeners = setupResizeListeners(host, syncViewportSize);

    return () => {
      disposed = true;
      removeResizeListeners();
      if (initialResizeFrame !== 0) {
        window.cancelAnimationFrame(initialResizeFrame);
      }
      app.canvas.removeEventListener('pointerdown', handlePointerDown);
      app.canvas.removeEventListener('pointermove', handlePointerMove);
      if (handlePointerUp) {
        app.canvas.removeEventListener('pointerup', handlePointerUp);
      }
      if (handlePointerCancel) {
        app.canvas.removeEventListener('pointercancel', handlePointerCancel);
      }
      app.canvas.removeEventListener('wheel', handleWheel);
      app.canvas.removeEventListener('contextmenu', handleContextMenu);
      middlePanRef.current = null;
      primaryDragRef.current = null;
      touchStateRef.current = { points: new Map(), gesture: null };
      canvasRef.current = null;
      layersRef.current = null;
      viewportRef.current?.destroy({ children: true });
      viewportRef.current = null;
      appRef.current?.destroy(true, { children: true });
      appRef.current = null;
    };
  }, [onPrimaryDragCancelRef, onPrimaryDragEndRef, onPrimaryDragMoveRef, onPrimaryDragStartRef]);

  useEffect(() => {
    const viewport = viewportRef.current;
    if (!viewport) {
      return;
    }

    const key = `${bounds.minX}:${bounds.minY}:${bounds.maxX}:${bounds.maxY}`;
    viewport.resize(viewport.screenWidth, viewport.screenHeight, worldWidth(bounds), worldHeight(bounds));
    if (fittedBoundsKeyRef.current !== key) {
      fitViewportToBounds(viewport, bounds, fitScaleRef);
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

  return hostRef;
}

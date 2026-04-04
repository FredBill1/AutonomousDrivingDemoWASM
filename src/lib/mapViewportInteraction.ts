import type { Viewport } from 'pixi-viewport';
import type React from 'react';

import type { ViewportConfig } from './appConfig';
import type { MapBoundingBox } from './mapServerNode';
import { clamp, setViewportTransform } from './mapViewportDraw';

export type ScreenPoint = {
  x: number;
  y: number;
};

export type TouchState = {
  points: Map<number, ScreenPoint>;
  gesture: {
    distance: number;
    centerX: number;
    centerY: number;
  } | null;
};

export function distance(a: ScreenPoint, b: ScreenPoint) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

type PointerHandlersParams = {
  viewportRef: React.RefObject<Viewport | null>;
  boundsRef: React.RefObject<MapBoundingBox>;
  hostRef: React.RefObject<HTMLDivElement | null>;
  primaryDragRef: React.RefObject<{ pointerId: number; pointerType: string } | null>;
  middlePanRef: React.RefObject<{ pointerId: number; lastX: number; lastY: number } | null>;
  touchStateRef: React.RefObject<TouchState>;
  fitScaleRef: React.RefObject<number>;
  canvasRef: React.RefObject<HTMLCanvasElement | null>;
  viewportConfigRef: React.RefObject<ViewportConfig>;
  onPrimaryDragStartRef: React.RefObject<(world: { x: number; y: number }) => boolean>;
  onPrimaryDragMoveRef: React.RefObject<(world: { x: number; y: number }) => void>;
  onPrimaryDragEndRef: React.RefObject<(world: { x: number; y: number }) => void>;
  onPrimaryDragCancelRef: React.RefObject<() => void>;
};

export function createPointerHandlers(params: PointerHandlersParams) {
  const {
    viewportRef,
    boundsRef,
    hostRef,
    primaryDragRef,
    middlePanRef,
    touchStateRef,
    fitScaleRef,
    canvasRef,
    viewportConfigRef,
    onPrimaryDragStartRef,
    onPrimaryDragMoveRef,
    onPrimaryDragEndRef,
    onPrimaryDragCancelRef,
  } = params;

  const toWorldFromClient = (clientX: number, clientY: number) => {
    const viewport = viewportRef.current;
    const currentHost = hostRef.current;
    if (!viewport || !currentHost) {
      return null;
    }

    const rect = currentHost.getBoundingClientRect();
    const local = viewport.toWorld(clientX - rect.left, clientY - rect.top);
    const currentBounds = boundsRef.current;
    return {
      x: currentBounds.minX + local.x,
      y: currentBounds.maxY - local.y,
    };
  };

  const cancelPrimaryDrag = () => {
    if (!primaryDragRef.current) {
      return;
    }
    primaryDragRef.current = null;
    onPrimaryDragCancelRef.current();
  };

  const handlePointerDown = (event: PointerEvent) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    if (event.pointerType === 'mouse' && event.button === 1) {
      event.preventDefault();
      middlePanRef.current = {
        pointerId: event.pointerId,
        lastX: event.clientX,
        lastY: event.clientY,
      };
      canvas.setPointerCapture(event.pointerId);
      return;
    }

    if (event.pointerType === 'touch') {
      touchStateRef.current.points.set(event.pointerId, { x: event.clientX, y: event.clientY });
      if (touchStateRef.current.points.size >= 2) {
        cancelPrimaryDrag();
        const [firstPoint, secondPoint] = Array.from(touchStateRef.current.points.values());
        touchStateRef.current.gesture = {
          distance: distance(firstPoint, secondPoint),
          centerX: (firstPoint.x + secondPoint.x) / 2,
          centerY: (firstPoint.y + secondPoint.y) / 2,
        };
        canvas.setPointerCapture(event.pointerId);
        return;
      }
    }

    const isPrimaryMouseButton = event.pointerType === 'mouse' && event.button === 0;
    const isSingleTouch = event.pointerType === 'touch' && touchStateRef.current.points.size === 1;
    if (!isPrimaryMouseButton && !isSingleTouch) {
      return;
    }

    const world = toWorldFromClient(event.clientX, event.clientY);
    if (!world || !onPrimaryDragStartRef.current(world)) {
      return;
    }

    primaryDragRef.current = { pointerId: event.pointerId, pointerType: event.pointerType };
    canvas.setPointerCapture(event.pointerId);
  };

  const handlePointerMove = (event: PointerEvent) => {
    if (event.pointerType === 'touch' && touchStateRef.current.points.has(event.pointerId)) {
      touchStateRef.current.points.set(event.pointerId, { x: event.clientX, y: event.clientY });
    }

    if (middlePanRef.current?.pointerId === event.pointerId) {
      event.preventDefault();
      const viewport = viewportRef.current;
      if (!viewport) {
        return;
      }
      viewport.position.x += event.clientX - middlePanRef.current.lastX;
      viewport.position.y += event.clientY - middlePanRef.current.lastY;
      middlePanRef.current = {
        pointerId: event.pointerId,
        lastX: event.clientX,
        lastY: event.clientY,
      };
      return;
    }

    if (touchStateRef.current.gesture && touchStateRef.current.points.size >= 2) {
      event.preventDefault();
      const viewport = viewportRef.current;
      const currentHost = hostRef.current;
      if (!viewport || !currentHost) {
        return;
      }

      const [firstPoint, secondPoint] = Array.from(touchStateRef.current.points.values());
      const nextCenterX = (firstPoint.x + secondPoint.x) / 2;
      const nextCenterY = (firstPoint.y + secondPoint.y) / 2;
      const nextDistance = distance(firstPoint, secondPoint);
      const previousGesture = touchStateRef.current.gesture;
      const { minZoom, maxZoom } = viewportConfigRef.current;

      viewport.position.x += nextCenterX - previousGesture.centerX;
      viewport.position.y += nextCenterY - previousGesture.centerY;

      const rect = currentHost.getBoundingClientRect();
      const screenX = nextCenterX - rect.left;
      const screenY = nextCenterY - rect.top;
      const worldPoint = viewport.toWorld(screenX, screenY);
      const scaleFactor = nextDistance / Math.max(previousGesture.distance, 1);
      const nextScale = clamp(viewport.scale.x * scaleFactor, minZoom, maxZoom);
      setViewportTransform(viewport, screenX, screenY, nextScale, worldPoint);

      touchStateRef.current.gesture = {
        distance: nextDistance,
        centerX: nextCenterX,
        centerY: nextCenterY,
      };
      return;
    }

    if (primaryDragRef.current?.pointerId !== event.pointerId) {
      return;
    }

    const world = toWorldFromClient(event.clientX, event.clientY);
    if (world) {
      onPrimaryDragMoveRef.current(world);
    }
  };

  const finishPointer = (event: PointerEvent, cancelled: boolean) => {
    if (middlePanRef.current?.pointerId === event.pointerId) {
      middlePanRef.current = null;
    }

    if (touchStateRef.current.points.has(event.pointerId)) {
      touchStateRef.current.points.delete(event.pointerId);
      if (touchStateRef.current.points.size < 2) {
        touchStateRef.current.gesture = null;
      }
    }

    if (primaryDragRef.current?.pointerId !== event.pointerId) {
      return;
    }

    primaryDragRef.current = null;
    if (cancelled) {
      onPrimaryDragCancelRef.current();
      return;
    }

    const world = toWorldFromClient(event.clientX, event.clientY);
    if (world) {
      onPrimaryDragEndRef.current(world);
    } else {
      onPrimaryDragCancelRef.current();
    }
  };

  const handleWheel = (event: WheelEvent) => {
    event.preventDefault();
    const viewport = viewportRef.current;
    const currentHost = hostRef.current;
    if (!viewport || !currentHost) {
      return;
    }

    const rect = currentHost.getBoundingClientRect();
    const screenX = event.clientX - rect.left;
    const screenY = event.clientY - rect.top;
    const worldPoint = viewport.toWorld(screenX, screenY);
    const { maxZoom, minZoom, minZoomRelativeToFit, wheelZoomSensitivity } = viewportConfigRef.current;
    const scaleFactor = Math.exp(-event.deltaY * wheelZoomSensitivity);
    const nextScale = clamp(
      viewport.scale.x * scaleFactor,
      Math.max(minZoom, fitScaleRef.current * minZoomRelativeToFit),
      maxZoom,
    );
    setViewportTransform(viewport, screenX, screenY, nextScale, worldPoint);
  };

  const handleContextMenu = (event: MouseEvent) => {
    event.preventDefault();
  };

  return {
    handlePointerDown,
    handlePointerMove,
    finishPointer,
    handleWheel,
    handleContextMenu,
  };
}

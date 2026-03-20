import { useEffect, useRef } from 'react'

import { Application, Graphics, Text } from 'pixi.js'
import { Viewport } from 'pixi-viewport'

import type { CarState, Mode, Obstacle } from '../lib/appModel'
import type { MapBoundingBox } from '../lib/mapServerNode'
import type { HybridAStarProgress, LocalPlannerPathPoint, LocalPlannerReferencePoint } from '../lib/wasmCore'
import type { CarShape, GoalUnreachableState, MotionLimits } from '../App'

type PathPoint = {
  x: number
  y: number
}

type MapViewportProps = {
  bounds: MapBoundingBox
  mode: Mode
  carShape: CarShape
  motionLimits: MotionLimits
  knownObstacles: Obstacle[]
  unknownObstacles: Obstacle[]
  car: CarState
  goal: CarState | null
  pressedPose: CarState | null
  goalUnreachable: GoalUnreachableState
  globalTrajectory: PathPoint[] | null
  localTrajectory: LocalPlannerPathPoint[]
  referencePoints: LocalPlannerReferencePoint[]
  globalPlannerSegments: HybridAStarProgress['segments'][]
  onPrimaryDragStart: (world: { x: number; y: number }) => boolean
  onPrimaryDragMove: (world: { x: number; y: number }) => void
  onPrimaryDragEnd: (world: { x: number; y: number }) => void
  onPrimaryDragCancel: () => void
}

type ScreenPoint = {
  x: number
  y: number
}

type TouchState = {
  points: Map<number, ScreenPoint>
  gesture: {
    distance: number
    centerX: number
    centerY: number
  } | null
}

type DrawLayers = {
  grid: Graphics
  boundary: Graphics
  segments: Graphics
  unknownObstacles: Graphics
  knownObstacles: Graphics
  globalTrajectory: Graphics
  localTrajectory: Graphics
  referencePoints: Graphics
  scanRing: Graphics
  cars: Graphics
  label: Text
}

const GRID_SPACING = 5
const MIN_ZOOM = 0.05
const MAX_ZOOM = 100

function worldWidth(bounds: MapBoundingBox) {
  return bounds.maxX - bounds.minX
}

function worldHeight(bounds: MapBoundingBox) {
  return bounds.maxY - bounds.minY
}

function toViewportX(x: number, bounds: MapBoundingBox) {
  return x - bounds.minX
}

function toViewportY(y: number, bounds: MapBoundingBox) {
  return bounds.maxY - y
}

function toViewportPoint(point: { x: number; y: number }, bounds: MapBoundingBox) {
  return {
    x: toViewportX(point.x, bounds),
    y: toViewportY(point.y, bounds),
  }
}

function distance(a: ScreenPoint, b: ScreenPoint) {
  return Math.hypot(a.x - b.x, a.y - b.y)
}

function clamp(value: number, minValue: number, maxValue: number) {
  return Math.min(maxValue, Math.max(minValue, value))
}

function drawPolyline(graphics: Graphics, points: PathPoint[], bounds: MapBoundingBox, width: number, color: number, alpha: number) {
  if (points.length < 2) {
    return
  }

  points.forEach((point, index) => {
    const viewPoint = toViewportPoint(point, bounds)
    if (index === 0) {
      graphics.moveTo(viewPoint.x, viewPoint.y)
    } else {
      graphics.lineTo(viewPoint.x, viewPoint.y)
    }
  })
  graphics.stroke({ width, color, alpha, cap: 'round', join: 'round' })
}

function transformCarPoint(car: Pick<CarState, 'x' | 'y' | 'yaw'>, localX: number, localY: number) {
  return {
    x: car.x + localX * Math.cos(car.yaw) - localY * Math.sin(car.yaw),
    y: car.y + localX * Math.sin(car.yaw) + localY * Math.cos(car.yaw),
  }
}

function buildCarPolygon(car: Pick<CarState, 'x' | 'y' | 'yaw'>, shape: CarShape) {
  const local = [
    [-shape.backToWheel, -shape.width / 2],
    [shape.length - shape.backToWheel, -shape.width / 2],
    [shape.length - shape.backToWheel, shape.width / 2],
    [-shape.backToWheel, shape.width / 2],
  ]

  return local.flatMap(([localX, localY]) => {
    const point = transformCarPoint(car, localX, localY)
    return [point.x, point.y]
  })
}

function buildWheelPolygons(car: Pick<CarState, 'x' | 'y' | 'yaw' | 'steer'>, shape: CarShape) {
  const halfWheelLength = shape.wheelLength / 2
  const halfWheelWidth = shape.wheelWidth / 2
  const wheelBox = [
    [-halfWheelLength, -halfWheelWidth],
    [halfWheelLength, -halfWheelWidth],
    [halfWheelLength, halfWheelWidth],
    [-halfWheelLength, halfWheelWidth],
  ]
  const cosSteer = Math.cos(car.steer)
  const sinSteer = Math.sin(car.steer)
  const rotateFrontWheelPoint = ([x, y]: number[]) => [x * cosSteer - y * sinSteer, x * sinSteer + y * cosSteer]

  const frontWheel = wheelBox.map(rotateFrontWheelPoint)
  const frontLeftWheel = frontWheel.map(([x, y]) => [x + shape.wheelBase, y + shape.wheelSpacing / 2])
  const frontRightWheel = frontWheel.map(([x, y]) => [x + shape.wheelBase, y - shape.wheelSpacing / 2])
  const rearLeftWheel = wheelBox.map(([x, y]) => [x, y + shape.wheelSpacing / 2])
  const rearRightWheel = wheelBox.map(([x, y]) => [x, y - shape.wheelSpacing / 2])

  return [frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel].map((wheel) =>
    wheel.flatMap(([localX, localY]) => {
      const point = transformCarPoint(car, localX, localY)
      return [point.x, point.y]
    }),
  )
}

function drawCar(
  graphics: Graphics,
  car: Pick<CarState, 'x' | 'y' | 'yaw' | 'steer'>,
  bounds: MapBoundingBox,
  shape: CarShape,
  strokeColor: number,
) {
  const bodyPolygon = buildCarPolygon(car, shape)
  const bodyPoints = bodyPolygon.flatMap((value, index) => (index % 2 === 0 ? [toViewportX(value, bounds)] : [toViewportY(value, bounds)]))
  graphics.poly(bodyPoints, true).fill({ color: 0xffffff, alpha: 0.08 }).stroke({ width: 0.2, color: strokeColor, alpha: 1 })

  const wheelPolygons = buildWheelPolygons(car, shape)
  wheelPolygons.forEach((wheelPolygon) => {
    const points = wheelPolygon.flatMap((value, index) => (index % 2 === 0 ? [toViewportX(value, bounds)] : [toViewportY(value, bounds)]))
    graphics.poly(points, true).stroke({ width: 0.15, color: strokeColor, alpha: 1 })
  })

  const rearAxle = toViewportPoint({ x: car.x, y: car.y }, bounds)
  graphics.circle(rearAxle.x, rearAxle.y, 0.12).fill({ color: strokeColor, alpha: 1 }).stroke({ width: 0.25, color: strokeColor, alpha: 1 })
}

function setViewportTransform(viewport: Viewport, screenX: number, screenY: number, scale: number, worldPoint: ScreenPoint) {
  viewport.setZoom(scale)
  viewport.position.set(screenX - worldPoint.x * viewport.scale.x, screenY - worldPoint.y * viewport.scale.y)
}

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
  const hostRef = useRef<HTMLDivElement | null>(null)
  const appRef = useRef<Application | null>(null)
  const viewportRef = useRef<Viewport | null>(null)
  const layersRef = useRef<DrawLayers | null>(null)
  const boundsRef = useRef(bounds)
  const onPrimaryDragStartRef = useRef(onPrimaryDragStart)
  const onPrimaryDragMoveRef = useRef(onPrimaryDragMove)
  const onPrimaryDragEndRef = useRef(onPrimaryDragEnd)
  const onPrimaryDragCancelRef = useRef(onPrimaryDragCancel)
  const middlePanRef = useRef<{ pointerId: number; lastX: number; lastY: number } | null>(null)
  const primaryDragRef = useRef<{ pointerId: number; pointerType: string } | null>(null)
  const touchStateRef = useRef<TouchState>({ points: new Map(), gesture: null })
  const fittedBoundsKeyRef = useRef<string | null>(null)
  const fitScaleRef = useRef(1)
  const drawRef = useRef<() => void>(() => { })

  useEffect(() => {
    boundsRef.current = bounds
  }, [bounds])

  useEffect(() => {
    onPrimaryDragStartRef.current = onPrimaryDragStart
    onPrimaryDragMoveRef.current = onPrimaryDragMove
    onPrimaryDragEndRef.current = onPrimaryDragEnd
    onPrimaryDragCancelRef.current = onPrimaryDragCancel
  }, [onPrimaryDragCancel, onPrimaryDragEnd, onPrimaryDragMove, onPrimaryDragStart])

  useEffect(() => {
    let disposed = false
    let handlePointerUp: ((event: PointerEvent) => void) | null = null
    let handlePointerCancel: ((event: PointerEvent) => void) | null = null
    const host = hostRef.current
    if (!host) {
      return
    }

    const width = Math.max(1, host.clientWidth)
    const height = Math.max(1, host.clientHeight)
    const app = new Application()

    const toWorldFromClient = (clientX: number, clientY: number) => {
      const viewport = viewportRef.current
      const currentHost = hostRef.current
      if (!viewport || !currentHost) {
        return null
      }

      const rect = currentHost.getBoundingClientRect()
      const local = viewport.toWorld(clientX - rect.left, clientY - rect.top)
      const currentBounds = boundsRef.current
      return {
        x: currentBounds.minX + local.x,
        y: currentBounds.maxY - local.y,
      }
    }

    const cancelPrimaryDrag = () => {
      if (!primaryDragRef.current) {
        return
      }
      primaryDragRef.current = null
      onPrimaryDragCancelRef.current()
    }

    const handlePointerDown = (event: PointerEvent) => {
      const canvas = app.canvas
      if (event.pointerType === 'mouse' && event.button === 1) {
        event.preventDefault()
        middlePanRef.current = { pointerId: event.pointerId, lastX: event.clientX, lastY: event.clientY }
        canvas.setPointerCapture(event.pointerId)
        return
      }

      if (event.pointerType === 'touch') {
        touchStateRef.current.points.set(event.pointerId, { x: event.clientX, y: event.clientY })
        if (touchStateRef.current.points.size >= 2) {
          cancelPrimaryDrag()
          const [firstPoint, secondPoint] = Array.from(touchStateRef.current.points.values())
          touchStateRef.current.gesture = {
            distance: distance(firstPoint, secondPoint),
            centerX: (firstPoint.x + secondPoint.x) / 2,
            centerY: (firstPoint.y + secondPoint.y) / 2,
          }
          canvas.setPointerCapture(event.pointerId)
          return
        }
      }

      const isPrimaryMouseButton = event.pointerType === 'mouse' && event.button === 0
      const isSingleTouch = event.pointerType === 'touch' && touchStateRef.current.points.size === 1
      if (!isPrimaryMouseButton && !isSingleTouch) {
        return
      }

      const world = toWorldFromClient(event.clientX, event.clientY)
      if (!world || !onPrimaryDragStartRef.current(world)) {
        return
      }

      primaryDragRef.current = { pointerId: event.pointerId, pointerType: event.pointerType }
      canvas.setPointerCapture(event.pointerId)
    }

    const handlePointerMove = (event: PointerEvent) => {
      if (event.pointerType === 'touch' && touchStateRef.current.points.has(event.pointerId)) {
        touchStateRef.current.points.set(event.pointerId, { x: event.clientX, y: event.clientY })
      }

      if (middlePanRef.current?.pointerId === event.pointerId) {
        event.preventDefault()
        const viewport = viewportRef.current
        if (!viewport) {
          return
        }
        viewport.position.x += event.clientX - middlePanRef.current.lastX
        viewport.position.y += event.clientY - middlePanRef.current.lastY
        middlePanRef.current = { pointerId: event.pointerId, lastX: event.clientX, lastY: event.clientY }
        return
      }

      if (touchStateRef.current.gesture && touchStateRef.current.points.size >= 2) {
        event.preventDefault()
        const viewport = viewportRef.current
        const currentHost = hostRef.current
        if (!viewport || !currentHost) {
          return
        }

        const [firstPoint, secondPoint] = Array.from(touchStateRef.current.points.values())
        const nextCenterX = (firstPoint.x + secondPoint.x) / 2
        const nextCenterY = (firstPoint.y + secondPoint.y) / 2
        const nextDistance = distance(firstPoint, secondPoint)
        const previousGesture = touchStateRef.current.gesture

        viewport.position.x += nextCenterX - previousGesture.centerX
        viewport.position.y += nextCenterY - previousGesture.centerY

        const rect = currentHost.getBoundingClientRect()
        const screenX = nextCenterX - rect.left
        const screenY = nextCenterY - rect.top
        const worldPoint = viewport.toWorld(screenX, screenY)
        const scaleFactor = nextDistance / Math.max(previousGesture.distance, 1)
        const nextScale = clamp(viewport.scale.x * scaleFactor, MIN_ZOOM, MAX_ZOOM)
        setViewportTransform(viewport, screenX, screenY, nextScale, worldPoint)

        touchStateRef.current.gesture = {
          distance: nextDistance,
          centerX: nextCenterX,
          centerY: nextCenterY,
        }
        return
      }

      if (primaryDragRef.current?.pointerId !== event.pointerId) {
        return
      }

      const world = toWorldFromClient(event.clientX, event.clientY)
      if (world) {
        onPrimaryDragMoveRef.current(world)
      }
    }

    const finishPointer = (event: PointerEvent, cancelled: boolean) => {
      if (middlePanRef.current?.pointerId === event.pointerId) {
        middlePanRef.current = null
      }

      if (touchStateRef.current.points.has(event.pointerId)) {
        touchStateRef.current.points.delete(event.pointerId)
        if (touchStateRef.current.points.size < 2) {
          touchStateRef.current.gesture = null
        }
      }

      if (primaryDragRef.current?.pointerId !== event.pointerId) {
        return
      }

      primaryDragRef.current = null
      if (cancelled) {
        onPrimaryDragCancelRef.current()
        return
      }

      const world = toWorldFromClient(event.clientX, event.clientY)
      if (world) {
        onPrimaryDragEndRef.current(world)
      } else {
        onPrimaryDragCancelRef.current()
      }
    }

    const handleWheel = (event: WheelEvent) => {
      event.preventDefault()
      const viewport = viewportRef.current
      const currentHost = hostRef.current
      if (!viewport || !currentHost) {
        return
      }

      const rect = currentHost.getBoundingClientRect()
      const screenX = event.clientX - rect.left
      const screenY = event.clientY - rect.top
      const worldPoint = viewport.toWorld(screenX, screenY)
      const scaleFactor = Math.exp(-event.deltaY * 0.0015)
      const nextScale = clamp(viewport.scale.x * scaleFactor, Math.max(MIN_ZOOM, fitScaleRef.current * 0.1), MAX_ZOOM)
      setViewportTransform(viewport, screenX, screenY, nextScale, worldPoint)
    }

    const handleContextMenu = (event: MouseEvent) => {
      event.preventDefault()
    }

    void app.init({
      width,
      height,
      antialias: true,
      autoDensity: true,
      backgroundAlpha: 0,
      preference: 'webgl',
      resolution: Math.min(window.devicePixelRatio || 1, 2),
    }).then(() => {
      if (disposed) {
        app.destroy(true, { children: true })
        return
      }

      app.canvas.classList.add('pixi-surface')
      host.appendChild(app.canvas)

      const viewport = new Viewport({
        screenWidth: width,
        screenHeight: height,
        worldWidth: worldWidth(boundsRef.current),
        worldHeight: worldHeight(boundsRef.current),
        events: app.renderer.events,
      })
      app.stage.addChild(viewport)

      const grid = new Graphics()
      const boundary = new Graphics()
      const segments = new Graphics()
      const unknownObstaclesLayer = new Graphics()
      const knownObstaclesLayer = new Graphics()
      const globalTrajectoryLayer = new Graphics()
      const localTrajectoryLayer = new Graphics()
      const referencePointsLayer = new Graphics()
      const scanRingLayer = new Graphics()
      const carsLayer = new Graphics()
      const label = new Text({
        text: 'Goal is unreachable',
        style: {
          fill: 0xff7b7b,
          fontFamily: 'Bahnschrift, Trebuchet MS, Segoe UI, sans-serif',
          fontSize: 18,
          fontWeight: '700',
          align: 'center',
        },
      })
      label.anchor.set(0.5)

      viewport.addChild(grid)
      viewport.addChild(boundary)
      viewport.addChild(segments)
      viewport.addChild(unknownObstaclesLayer)
      viewport.addChild(knownObstaclesLayer)
      viewport.addChild(globalTrajectoryLayer)
      viewport.addChild(localTrajectoryLayer)
      viewport.addChild(referencePointsLayer)
      viewport.addChild(scanRingLayer)
      viewport.addChild(carsLayer)
      app.stage.addChild(label)

      appRef.current = app
      viewportRef.current = viewport
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
      }

      handlePointerUp = (event: PointerEvent) => finishPointer(event, false)
      handlePointerCancel = (event: PointerEvent) => finishPointer(event, true)

      app.canvas.addEventListener('pointerdown', handlePointerDown)
      app.canvas.addEventListener('pointermove', handlePointerMove)
      app.canvas.addEventListener('pointerup', handlePointerUp)
      app.canvas.addEventListener('pointercancel', handlePointerCancel)
      app.canvas.addEventListener('wheel', handleWheel, { passive: false })
      app.canvas.addEventListener('contextmenu', handleContextMenu)

      drawRef.current()
    })

    const resizeObserver = new ResizeObserver(() => {
      const currentApp = appRef.current
      const viewport = viewportRef.current
      const currentHost = hostRef.current
      if (!currentApp || !viewport || !currentHost) {
        return
      }

      const nextWidth = Math.max(1, currentHost.clientWidth)
      const nextHeight = Math.max(1, currentHost.clientHeight)
      const currentBounds = boundsRef.current
      currentApp.renderer.resize(nextWidth, nextHeight)
      viewport.resize(nextWidth, nextHeight, worldWidth(currentBounds), worldHeight(currentBounds))

      if (fittedBoundsKeyRef.current === null) {
        const scale = Math.min(nextWidth / Math.max(worldWidth(currentBounds), 1), nextHeight / Math.max(worldHeight(currentBounds), 1))
        fitScaleRef.current = scale
        viewport.setZoom(scale)
        viewport.position.set((nextWidth - worldWidth(currentBounds) * scale) / 2, (nextHeight - worldHeight(currentBounds) * scale) / 2)
        fittedBoundsKeyRef.current = `${currentBounds.minX}:${currentBounds.minY}:${currentBounds.maxX}:${currentBounds.maxY}`
      }
    })
    resizeObserver.observe(host)

    return () => {
      disposed = true
      resizeObserver.disconnect()
      app.canvas.removeEventListener('pointerdown', handlePointerDown)
      app.canvas.removeEventListener('pointermove', handlePointerMove)
      if (handlePointerUp) {
        app.canvas.removeEventListener('pointerup', handlePointerUp)
      }
      if (handlePointerCancel) {
        app.canvas.removeEventListener('pointercancel', handlePointerCancel)
      }
      app.canvas.removeEventListener('wheel', handleWheel)
      app.canvas.removeEventListener('contextmenu', handleContextMenu)
      middlePanRef.current = null
      primaryDragRef.current = null
      touchStateRef.current = { points: new Map(), gesture: null }
      layersRef.current = null
      viewportRef.current?.destroy({ children: true })
      viewportRef.current = null
      appRef.current?.destroy(true, { children: true })
      appRef.current = null
    }
  }, [])

  useEffect(() => {
    const viewport = viewportRef.current
    if (!viewport) {
      return
    }

    const key = `${bounds.minX}:${bounds.minY}:${bounds.maxX}:${bounds.maxY}`
    const width = worldWidth(bounds)
    const height = worldHeight(bounds)
    viewport.resize(viewport.screenWidth, viewport.screenHeight, width, height)
    if (fittedBoundsKeyRef.current !== key) {
      const scale = Math.min(viewport.screenWidth / Math.max(width, 1), viewport.screenHeight / Math.max(height, 1))
      fitScaleRef.current = scale
      viewport.setZoom(scale)
      viewport.position.set((viewport.screenWidth - width * scale) / 2, (viewport.screenHeight - height * scale) / 2)
      fittedBoundsKeyRef.current = key
    }
  }, [bounds])

  useEffect(() => {
    drawRef.current = () => {
      const layers = layersRef.current
      if (!layers) {
        return
      }

      layers.grid.clear()
      const width = worldWidth(bounds)
      const height = worldHeight(bounds)
      for (let x = 0; x <= width; x += GRID_SPACING) {
        layers.grid.moveTo(x, 0)
        layers.grid.lineTo(x, height)
      }
      for (let y = 0; y <= height; y += GRID_SPACING) {
        layers.grid.moveTo(0, y)
        layers.grid.lineTo(width, y)
      }
      layers.grid.stroke({ width: 0.1, color: 0xffffff, alpha: 0.08 })

      layers.boundary.clear()
      layers.boundary.rect(0, 0, width, height).stroke({ width: 0.3, color: 0xffffff, alpha: 0.18 })

      layers.segments.clear()
      globalPlannerSegments.forEach((segmentGroup) => {
        segmentGroup.forEach((segment) => {
          layers.segments.moveTo(toViewportX(segment.x1, bounds), toViewportY(segment.y1, bounds))
          layers.segments.lineTo(toViewportX(segment.x2, bounds), toViewportY(segment.y2, bounds))
        })
      })
      layers.segments.stroke({ width: 0.1, color: 0x57d8ff, alpha: 0.18 })

      layers.unknownObstacles.clear()
      unknownObstacles.forEach((obstacle) => {
        const point = toViewportPoint(obstacle, bounds)
        layers.unknownObstacles.circle(point.x, point.y, 0.28).fill({ color: 0x57d8ff, alpha: 0.95 })
      })

      layers.knownObstacles.clear()
      knownObstacles.forEach((obstacle) => {
        const point = toViewportPoint(obstacle, bounds)
        layers.knownObstacles.circle(point.x, point.y, 0.24).fill({ color: 0xff6f6f, alpha: 0.95 })
      })

      layers.globalTrajectory.clear()
      if (globalTrajectory) {
        drawPolyline(layers.globalTrajectory, globalTrajectory, bounds, 0.3, 0x57d8ff, 0.95)
      }

      layers.localTrajectory.clear()
      drawPolyline(layers.localTrajectory, localTrajectory, bounds, 0.35, 0x9fe870, 0.96)

      layers.referencePoints.clear()
      referencePoints.forEach((point) => {
        const viewPoint = toViewportPoint(point, bounds)
        layers.referencePoints.moveTo(viewPoint.x - 0.35, viewPoint.y - 0.35)
        layers.referencePoints.lineTo(viewPoint.x + 0.35, viewPoint.y + 0.35)
        layers.referencePoints.moveTo(viewPoint.x - 0.35, viewPoint.y + 0.35)
        layers.referencePoints.lineTo(viewPoint.x + 0.35, viewPoint.y - 0.35)
      })
      layers.referencePoints.stroke({ width: 0.25, color: 0xff7a7a, alpha: 0.94 })

      layers.scanRing.clear()
      const scanCenter = toViewportPoint(
        {
          x: car.x + carShape.backToCenter * Math.cos(car.yaw),
          y: car.y + carShape.backToCenter * Math.sin(car.yaw),
        },
        bounds,
      )
      layers.scanRing
        .circle(scanCenter.x, scanCenter.y, motionLimits.scanRadius)
        .fill({ color: 0x57d8ff, alpha: 0.05 })
        .stroke({ width: 0.24, color: 0x57d8ff, alpha: 0.32 })

      layers.cars.clear()
      drawCar(layers.cars, car, bounds, carShape, 0xffffff)
      if (goal) {
        drawCar(layers.cars, goal, bounds, carShape, 0x9fe870)
      }
      if (pressedPose) {
        drawCar(layers.cars, pressedPose, bounds, carShape, mode === 'pose' ? 0xffffff : 0x9fe870)
      }

      layers.label.visible = goalUnreachable.visible
      if (goalUnreachable.visible) {
        const labelPoint = toViewportPoint(goalUnreachable, bounds)
        const viewport = viewportRef.current
        if (!viewport) {
          return
        }
        const screenPoint = viewport.toScreen(labelPoint)
        layers.label.position.set(screenPoint.x, screenPoint.y)
      }
    }

    drawRef.current()
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
    motionLimits.scanRadius,
    pressedPose,
    referencePoints,
    unknownObstacles,
  ])

  return <div ref={hostRef} className="map-view" />
}

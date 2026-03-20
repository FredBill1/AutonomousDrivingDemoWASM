import { useEffect, useRef } from 'react'

import { Application, Graphics } from 'pixi.js'

type HistoryPoint = {
  t: number
  value: number
}

type HistoryChartProps = {
  points: HistoryPoint[]
  minValue: number
  maxValue: number
  lineColor: number
}

const MARGIN_LEFT = 18
const MARGIN_RIGHT = 10
const MARGIN_TOP = 10
const MARGIN_BOTTOM = 16

function clampRange(value: number) {
  return Math.max(value, 0.001)
}

export function HistoryChart({ points, minValue, maxValue, lineColor }: HistoryChartProps) {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const appRef = useRef<Application | null>(null)
  const frameRef = useRef<Graphics | null>(null)
  const lineRef = useRef<Graphics | null>(null)
  const drawRef = useRef<() => void>(() => {})

  useEffect(() => {
    let disposed = false
    const host = hostRef.current
    if (!host) {
      return
    }

    const width = Math.max(1, host.clientWidth)
    const height = Math.max(1, host.clientHeight)
    const app = new Application()

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

      const frame = new Graphics()
      const line = new Graphics()
      app.stage.addChild(frame)
      app.stage.addChild(line)

      appRef.current = app
      frameRef.current = frame
      lineRef.current = line
      drawRef.current()
    })

    const resizeObserver = new ResizeObserver(() => {
      const currentApp = appRef.current
      const currentHost = hostRef.current
      if (!currentApp || !currentHost) {
        return
      }

      currentApp.renderer.resize(Math.max(1, currentHost.clientWidth), Math.max(1, currentHost.clientHeight))
      drawRef.current()
    })
    resizeObserver.observe(host)

    return () => {
      disposed = true
      resizeObserver.disconnect()
      lineRef.current = null
      frameRef.current = null
      appRef.current?.destroy(true, { children: true })
      appRef.current = null
    }
  }, [])

  useEffect(() => {
    drawRef.current = () => {
      const app = appRef.current
      const frame = frameRef.current
      const line = lineRef.current
      if (!app || !frame || !line) {
        return
      }

      const width = app.renderer.width
      const height = app.renderer.height
      const plotWidth = Math.max(1, width - MARGIN_LEFT - MARGIN_RIGHT)
      const plotHeight = Math.max(1, height - MARGIN_TOP - MARGIN_BOTTOM)
      const minT = points[0]?.t ?? 0
      const maxT = points[points.length - 1]?.t ?? minT + 1
      const tRange = clampRange(maxT - minT)
      const valueRange = clampRange(maxValue - minValue)
      const baseX = MARGIN_LEFT
      const baseY = MARGIN_TOP

      frame.clear()
      frame
        .moveTo(baseX, baseY)
        .lineTo(baseX, baseY + plotHeight)
        .lineTo(baseX + plotWidth, baseY + plotHeight)
        .stroke({ width: 1.2, color: 0xffffff, alpha: 0.18 })

      line.clear()
      if (points.length < 2) {
        return
      }

      points.forEach((point, index) => {
        const x = baseX + ((point.t - minT) / tRange) * plotWidth
        const y = baseY + plotHeight - ((point.value - minValue) / valueRange) * plotHeight
        if (index === 0) {
          line.moveTo(x, y)
        } else {
          line.lineTo(x, y)
        }
      })
      line.stroke({ width: 2, color: lineColor, alpha: 1, cap: 'round', join: 'round' })
    }

    drawRef.current()
  }, [lineColor, maxValue, minValue, points])

  return <div ref={hostRef} className="chart-view" />
}

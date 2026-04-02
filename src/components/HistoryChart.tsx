import { useCallback, useEffect, useRef } from 'react';

import { Container, Graphics, Text, type Application } from 'pixi.js';

import { usePixiLifecycle } from '../hooks/usePixiLifecycle';

type HistoryPoint = {
  t: number;
  value: number;
};

type HistoryChartProps = {
  points: HistoryPoint[];
  minValue: number;
  maxValue: number;
  lineColor: number;
};

const MARGIN_LEFT = 42;
const MARGIN_RIGHT = 12;
const MARGIN_TOP = 10;
const MARGIN_BOTTOM = 26;
const TICK_SIZE = 4;
const LABEL_COLOR = 0xc7d6da;
const GRID_COLOR = 0xffffff;
const GRID_ALPHA = 0.08;
const AXIS_ALPHA = 0.18;

function destroyContainerChildren(container: Container) {
  container.removeChildren().forEach((child) => child.destroy());
}

function clampRange(value: number) {
  return Math.max(value, 0.001);
}

function getNiceStep(rawStep: number) {
  const exponent = Math.floor(Math.log10(rawStep));
  const fraction = rawStep / 10 ** exponent;

  if (fraction <= 1) {
    return 10 ** exponent;
  }
  if (fraction <= 2) {
    return 2 * 10 ** exponent;
  }
  if (fraction <= 5) {
    return 5 * 10 ** exponent;
  }
  return 10 * 10 ** exponent;
}

function getTickDecimals(step: number) {
  if (step >= 1) {
    return 0;
  }
  return Math.min(3, Math.max(0, Math.ceil(-Math.log10(step))));
}

function formatTickValue(value: number, step: number) {
  const decimals = getTickDecimals(step);
  if (Math.abs(value) < step * 0.5 * 10 ** -decimals) {
    return '0';
  }
  return value.toFixed(decimals);
}

function buildTicks(min: number, max: number, maxTickCount: number) {
  const range = clampRange(max - min);
  const safeTickCount = Math.max(2, maxTickCount);
  const step = getNiceStep(range / (safeTickCount - 1));
  const start = Math.ceil(min / step) * step;
  const end = Math.floor(max / step) * step;
  const ticks: Array<{ value: number; label: string }> = [];

  for (let value = start; value <= end + step * 0.5; value += step) {
    const normalizedValue = Math.abs(value) < step * 1e-6 ? 0 : value;
    ticks.push({ value: normalizedValue, label: formatTickValue(normalizedValue, step) });
  }

  if (ticks.length === 0) {
    ticks.push({ value: min, label: formatTickValue(min, step) });
    ticks.push({ value: max, label: formatTickValue(max, step) });
  }

  return ticks;
}

export function HistoryChart({ points, minValue, maxValue, lineColor }: HistoryChartProps) {
  const hostRef = useRef<HTMLDivElement | null>(null);
  const appRef = useRef<Application | null>(null);
  const frameRef = useRef<Graphics | null>(null);
  const lineRef = useRef<Graphics | null>(null);
  const labelsRef = useRef<Container | null>(null);
  const drawRef = useRef<() => void>(() => {});

  usePixiLifecycle(
    hostRef,
    useCallback(({ app }: { app: Application }) => {
      const frame = new Graphics();
      const line = new Graphics();
      const labels = new Container();
      app.stage.addChild(frame);
      app.stage.addChild(line);
      app.stage.addChild(labels);

      appRef.current = app;
      frameRef.current = frame;
      lineRef.current = line;
      labelsRef.current = labels;

      return {
        handleResize: () => {
          drawRef.current();
        },
        cleanup: () => {
          lineRef.current = null;
          frameRef.current = null;
          labelsRef.current = null;
          appRef.current = null;
        },
      };
    }, []),
  );

  useEffect(() => {
    drawRef.current = () => {
      const app = appRef.current;
      const frame = frameRef.current;
      const line = lineRef.current;
      const labels = labelsRef.current;
      if (!app || !frame || !line || !labels) {
        return;
      }

      const width = app.renderer.width;
      const height = app.renderer.height;
      const plotWidth = Math.max(1, width - MARGIN_LEFT - MARGIN_RIGHT);
      const plotHeight = Math.max(1, height - MARGIN_TOP - MARGIN_BOTTOM);
      const minT = points[0]?.t ?? 0;
      const maxT = points[points.length - 1]?.t ?? minT + 1;
      const tRange = clampRange(maxT - minT);
      const valueRange = clampRange(maxValue - minValue);
      const baseX = MARGIN_LEFT;
      const baseY = MARGIN_TOP;
      const yTicks = buildTicks(minValue, maxValue, Math.max(3, Math.floor(plotHeight / 32)));
      const xTicks = buildTicks(minT, maxT, Math.max(3, Math.floor(plotWidth / 70)));

      frame.clear();
      destroyContainerChildren(labels);

      const createTickLabel = (text: string): Text =>
        new Text({
          text,
          style: {
            fontFamily: 'Bahnschrift, Trebuchet MS, Segoe UI, sans-serif',
            fontSize: 10,
            fill: LABEL_COLOR,
          },
        });

      yTicks.forEach((tick) => {
        const y = baseY + plotHeight - ((tick.value - minValue) / valueRange) * plotHeight;
        frame
          .moveTo(baseX, y)
          .lineTo(baseX + plotWidth, y)
          .moveTo(baseX - TICK_SIZE, y)
          .lineTo(baseX, y);

        const label = createTickLabel(tick.label);
        label.anchor.set(1, 0.5);
        label.position.set(baseX - TICK_SIZE - 4, y);
        labels.addChild(label);
      });

      xTicks.forEach((tick) => {
        const x = baseX + ((tick.value - minT) / tRange) * plotWidth;
        frame
          .moveTo(x, baseY)
          .lineTo(x, baseY + plotHeight)
          .moveTo(x, baseY + plotHeight)
          .lineTo(x, baseY + plotHeight + TICK_SIZE);

        const label = createTickLabel(tick.label);
        label.anchor.set(0.5, 0);
        label.position.set(x, baseY + plotHeight + TICK_SIZE + 2);
        labels.addChild(label);
      });

      const strokeFrame = (strokeWidth: number, alpha: number) => {
        frame
          .moveTo(baseX, baseY)
          .lineTo(baseX, baseY + plotHeight)
          .lineTo(baseX + plotWidth, baseY + plotHeight)
          .stroke({ width: strokeWidth, color: GRID_COLOR, alpha });
      };

      strokeFrame(1, GRID_ALPHA);
      strokeFrame(1.2, AXIS_ALPHA);

      line.clear();
      if (points.length < 2) {
        return;
      }

      points.forEach((point, index) => {
        const x = baseX + ((point.t - minT) / tRange) * plotWidth;
        const y = baseY + plotHeight - ((point.value - minValue) / valueRange) * plotHeight;
        if (index === 0) {
          line.moveTo(x, y);
        } else {
          line.lineTo(x, y);
        }
      });
      line.stroke({ width: 2, color: lineColor, alpha: 1, cap: 'round', join: 'round' });
    };

    drawRef.current();
  }, [lineColor, maxValue, minValue, points]);

  return <div ref={hostRef} className="chart-view" />;
}

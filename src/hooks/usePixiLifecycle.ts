import { Application } from 'pixi.js';
import { useEffect } from 'react';

import { setupPixiCanvas, setupResizeListeners, syncCanvasElementSize } from '../lib/pixiAppInit';

type PixiReadyResult = {
  handleResize: () => void;
  cleanup?: () => void;
};

export function usePixiLifecycle(
  hostRef: React.RefObject<HTMLDivElement | null>,
  onReady: (context: { app: Application; host: HTMLDivElement }) => PixiReadyResult,
) {
  useEffect(() => {
    let disposed = false;
    let removeResizeListeners: (() => void) | null = null;
    let initialResizeFrame = 0;
    let teardown: (() => void) | undefined;
    const host = hostRef.current;
    if (!host) {
      return;
    }

    const app = new Application();
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);

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

        const { cleanup, handleResize } = onReady({ app, host });
        const resize = () => {
          if (disposed) {
            return;
          }
          const nextWidth = Math.max(1, host.clientWidth);
          const nextHeight = Math.max(1, host.clientHeight);
          app.renderer.resize(nextWidth, nextHeight);
          syncCanvasElementSize(app.canvas, nextWidth, nextHeight);
          handleResize();
        };

        resize();
        initialResizeFrame = window.requestAnimationFrame(resize);
        removeResizeListeners = setupResizeListeners(host, resize);

        teardown = cleanup ?? undefined;
      })
      .catch(() => {});

    return () => {
      disposed = true;
      if (initialResizeFrame !== 0) {
        window.cancelAnimationFrame(initialResizeFrame);
      }
      removeResizeListeners?.();
      teardown?.();
      app.destroy(true, { children: true });
    };
  }, [hostRef, onReady]);
}

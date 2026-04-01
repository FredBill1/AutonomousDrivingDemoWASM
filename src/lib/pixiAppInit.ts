import { Application } from 'pixi.js';

export function syncCanvasElementSize(canvas: HTMLCanvasElement, width: number, height: number) {
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
}

export function setupPixiCanvas(
    app: Application,
    host: HTMLElement,
    width: number,
    height: number,
    disposed: boolean,
): boolean {
    if (disposed) {
        app.destroy(true, { children: true });
        return false;
    }
    app.canvas.classList.add('pixi-surface');
    syncCanvasElementSize(app.canvas, width, height);
    host.appendChild(app.canvas);
    return true;
}

export function setupResizeListeners(host: HTMLElement, onResize: () => void): () => void {
    const resizeObserver = new ResizeObserver(onResize);
    resizeObserver.observe(host);
    window.addEventListener('resize', onResize);
    window.visualViewport?.addEventListener('resize', onResize);
    return () => {
        resizeObserver.disconnect();
        window.removeEventListener('resize', onResize);
        window.visualViewport?.removeEventListener('resize', onResize);
    };
}

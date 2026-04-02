import { handlers } from './computeWorkerHandlers';
import type { WorkerRequest, WorkerResponse } from './workerTypes';

self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const message = event.data;
  const handler = handlers[message.type as keyof typeof handlers] as ((payload: never) => Promise<unknown>) | undefined;

  if (!handler) {
    self.postMessage({
      id: message.id,
      ok: false,
      error: `Unknown worker request: ${message.type}`,
    } satisfies WorkerResponse);
    return;
  }

  void handler(message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse);
    });
};

export {};

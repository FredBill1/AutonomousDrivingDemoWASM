import { handlers } from './computeWorkerHandlers';
import type { WorkerHandlerMap, WorkerRequest, WorkerResponse } from './workerTypes';

function isKnownRequestType(type: string): type is keyof WorkerHandlerMap {
  return type in handlers;
}

self.onmessage = (event: MessageEvent<WorkerRequest | { id?: number; type?: string; payload?: unknown }>) => {
  const message = event.data;
  if (typeof message.id !== 'number' || typeof message.type !== 'string' || !isKnownRequestType(message.type)) {
    self.postMessage({
      id: typeof message.id === 'number' ? message.id : -1,
      ok: false,
      error: `Unknown worker request: ${String(message.type)}`,
    } satisfies WorkerResponse);
    return;
  }

  const requestId = message.id;
  const handler: WorkerHandlerMap[typeof message.type] = handlers[message.type];

  void handler(message.payload as never)
    .then((result) => {
      self.postMessage({ id: requestId, ok: true, result } satisfies WorkerResponse);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: requestId, ok: false, error: errorMessage } satisfies WorkerResponse);
    });
};

export {};

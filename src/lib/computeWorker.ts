import { handlers } from './computeWorkerHandlers';
import type { WorkerHandlerMap, WorkerRequest, WorkerResponse } from './workerTypes';

async function handleRequest(message: WorkerRequest) {
  const handler: WorkerHandlerMap[typeof message.type] = handlers[message.type];
  return handler(message.payload as never);
}

self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const message = event.data;

  void handleRequest(message)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse);
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error);
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse);
    });
};

export {};

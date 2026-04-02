type PendingRequest = {
  resolve: (value: unknown) => void;
  reject: (reason?: unknown) => void;
};

type RpcMethodMap = Record<string, { payload: unknown; result: unknown }>;
type EventMap = Record<string, unknown>;
type PayloadArgs<Payload> = undefined extends Payload ? [] | [payload: Payload] : [payload: Payload];

type RpcResponse<Methods extends RpcMethodMap> = {
  [Key in keyof Methods]:
    | { id: number; ok: true; result: Methods[Key]['result'] }
    | { id: number; ok: false; error: string };
}[keyof Methods];

type RpcEvent<Events extends EventMap> = {
  [Key in keyof Events]: { type: Key; payload: Events[Key] };
}[keyof Events];

function isWorkerEvent<Events extends EventMap>(message: unknown): message is RpcEvent<Events> {
  return typeof message === 'object' && message !== null && 'type' in message && !('id' in message);
}

function isRpcResponse<Methods extends RpcMethodMap>(message: unknown): message is RpcResponse<Methods> {
  return typeof message === 'object' && message !== null && 'id' in message && 'ok' in message;
}

export function createWorkerRpc<Methods extends RpcMethodMap, Events extends EventMap>(
  workerFactory: () => Worker,
  onEvent?: (event: RpcEvent<Events>) => void,
) {
  let worker: Worker | null = null;
  let nextId = 1;
  const pending = new Map<number, PendingRequest>();

  const rejectPending = (message: string) => {
    for (const request of pending.values()) {
      request.reject(new Error(message));
    }
    pending.clear();
  };

  const ensureWorker = () => {
    if (worker) {
      return worker;
    }

    worker = workerFactory();
    worker.onmessage = (event: MessageEvent<RpcResponse<Methods> | RpcEvent<Events>>) => {
      const message = event.data;
      if (isWorkerEvent<Events>(message)) {
        onEvent?.(message);
        return;
      }

      if (!isRpcResponse<Methods>(message)) {
        return;
      }

      const request = pending.get(message.id);
      if (!request) {
        return;
      }
      pending.delete(message.id);

      if (message.ok) {
        request.resolve(message.result);
        return;
      }

      request.reject(new Error(message.error));
    };
    worker.onerror = () => {
      rejectPending('Worker execution failed');
      worker?.terminate();
      worker = null;
    };
    worker.onmessageerror = () => {
      rejectPending('Worker message handling failed');
      worker?.terminate();
      worker = null;
    };

    return worker;
  };

  return {
    call<Key extends keyof Methods & string>(type: Key, ...args: PayloadArgs<Methods[Key]['payload']>) {
      const activeWorker = ensureWorker();
      const id = nextId;
      nextId += 1;
      const payload = args[0];

      return new Promise<Methods[Key]['result']>((resolve, reject) => {
        pending.set(id, { resolve: (value) => resolve(value as Methods[Key]['result']), reject });
        activeWorker.postMessage(payload === undefined ? { id, type } : { id, type, payload });
      });
    },

    reset(reason = 'Worker reset') {
      rejectPending(reason);
      worker?.terminate();
      worker = null;
    },
  };
}

type PendingRequest = {
  resolve: (value: any) => void
  reject: (reason?: unknown) => void
}

type RpcResponse =
  | { id: number; ok: true; result: unknown }
  | { id: number; ok: false; error: string }

type WorkerMessage = RpcResponse | { type: string; payload?: unknown }

function isWorkerEvent(message: WorkerMessage): message is { type: string; payload?: unknown } {
  return 'type' in message && !('id' in message)
}

function isRpcResponse(message: WorkerMessage): message is RpcResponse {
  return 'id' in message && typeof message.id === 'number' && 'ok' in message
}

export function createWorkerRpc(workerFactory: () => Worker, onEvent?: (message: { type: string; payload?: unknown }) => void) {
  let worker: Worker | null = null
  let nextId = 1
  const pending = new Map<number, PendingRequest>()

  const rejectPending = (message: string) => {
    for (const request of pending.values()) {
      request.reject(new Error(message))
    }
    pending.clear()
  }

  const ensureWorker = () => {
    if (worker) return worker

    worker = workerFactory()
    worker.onmessage = (event: MessageEvent<WorkerMessage>) => {
      const message = event.data
      if (isWorkerEvent(message)) {
        onEvent?.(message)
        return
      }

      if (!isRpcResponse(message)) {
        return
      }

      const request = pending.get(message.id)
      if (!request) return
      pending.delete(message.id)

      if (message.ok) {
        request.resolve(message.result)
        return
      }

      request.reject(new Error(message.error))
    }
    worker.onerror = () => {
      rejectPending('Worker execution failed')
      worker?.terminate()
      worker = null
    }
    worker.onmessageerror = () => {
      rejectPending('Worker message handling failed')
      worker?.terminate()
      worker = null
    }

    return worker
  }

  return {
    call<T>(type: string, payload?: unknown): Promise<T> {
      const activeWorker = ensureWorker()
      const id = nextId
      nextId += 1

      return new Promise<T>((resolve, reject) => {
        pending.set(id, { resolve, reject })
        activeWorker.postMessage({ id, type, payload })
      })
    },

    reset(reason = 'Worker reset') {
      rejectPending(reason)
      worker?.terminate()
      worker = null
    },
  }
}

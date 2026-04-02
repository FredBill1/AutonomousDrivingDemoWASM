export type WasmDisposable = {
  free(): void;
};

export function disposeWasmResource(resource: WasmDisposable | null | undefined) {
  resource?.free();
}

export function usingWasmResource<T extends WasmDisposable, TResult>(
  resource: T,
  run: (resource: T) => TResult,
): TResult {
  try {
    return run(resource);
  } finally {
    resource.free();
  }
}

export function usingWasmPair<TFirst extends WasmDisposable, TSecond extends WasmDisposable, TResult>(
  first: TFirst,
  second: TSecond,
  run: (first: TFirst, second: TSecond) => TResult,
): TResult {
  try {
    return run(first, second);
  } finally {
    second.free();
    first.free();
  }
}

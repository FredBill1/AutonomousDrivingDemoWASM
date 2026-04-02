export type WasmDisposable = {
  free(): void;
};

export function disposeWasmResource(resource: WasmDisposable | null | undefined) {
  resource?.free();
}

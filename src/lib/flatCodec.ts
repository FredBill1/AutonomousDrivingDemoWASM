export function assertFlatTupleLength(values: ArrayLike<number>, stride: number, label: string) {
  if (values.length % stride !== 0) {
    throw new Error(`${label} length must be a multiple of ${stride}`);
  }
}

export function decodeFlatTuples<T>(
  values: ArrayLike<number>,
  stride: number,
  label: string,
  decode: (values: ArrayLike<number>, offset: number) => T,
) {
  assertFlatTupleLength(values, stride, label);

  const tuples: T[] = [];
  for (let offset = 0; offset < values.length; offset += stride) {
    tuples.push(decode(values, offset));
  }
  return tuples;
}

export function encodeFlatTuples<T>(items: T[], encode: (item: T) => readonly number[]) {
  const values: number[] = [];
  for (const item of items) {
    values.push(...encode(item));
  }
  return values;
}

export function encodeFlatTuplesToFloat64<T>(items: T[], encode: (item: T) => readonly number[]) {
  return Float64Array.from(encodeFlatTuples(items, encode));
}

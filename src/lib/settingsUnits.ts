import { MS_TO_KMH, RAD_TO_DEG } from './constants';

export const SETTINGS_DISPLAY_VALUE_DIGITS = 6;

export type SettingsValueFormat = {
  unit: string;
  toDisplay: (value: number) => number;
  toStored: (value: number) => number;
};

function roundDisplayValue(value: number) {
  return Number(value.toFixed(SETTINGS_DISPLAY_VALUE_DIGITS));
}

function createIdentityFormat(unit: string): SettingsValueFormat {
  return {
    unit,
    toDisplay: (value) => value,
    toStored: (value) => value,
  };
}

function createLinearFormat(unit: string, toDisplayFactor: number): SettingsValueFormat {
  return {
    unit,
    toDisplay: (value) => roundDisplayValue(value * toDisplayFactor),
    toStored: (value) => value / toDisplayFactor,
  };
}

export const METERS = createIdentityFormat('m');
export const METERS_PER_SECOND_SQUARED = createIdentityFormat('m/s²');
export const SECONDS = createIdentityFormat('s');
export const MILLISECONDS = createIdentityFormat('ms');
export const KILOMETERS_PER_HOUR_IDENTITY = createIdentityFormat('km/h');
export const KILOMETERS_PER_HOUR = createLinearFormat('km/h', MS_TO_KMH);
export const DEGREES = createLinearFormat('deg', RAD_TO_DEG);
export const DEGREES_PER_SECOND = createLinearFormat('deg/s', RAD_TO_DEG);
export const COUNT = createIdentityFormat('count');
export const STEPS = createIdentityFormat('steps');
export const ITERATIONS = createIdentityFormat('iterations');
export const BATCHES = createIdentityFormat('batches');
export const SEGMENTS = createIdentityFormat('segments');
export const COST = createIdentityFormat('cost');
export const SCALE = createIdentityFormat('scale');
export const RATIO = createIdentityFormat('ratio');
export const THRESHOLD = createIdentityFormat('threshold');

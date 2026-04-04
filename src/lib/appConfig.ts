import { MIN_RANGE_VALUE } from './constants';
import { cloneControllerConfig, getDefaultControllerConfig } from './controllerConfig';
import type { ControllerConfig } from './workerContracts';

const APP_CONFIG_STORAGE_KEY = 'autonomous-driving-demo-settings-v1';

export type UiConfig = {
  replanMaxSpeedKmh: number;
  maxGlobalPlannerDisplayBatches: number;
  minZoom: number;
  maxZoom: number;
  minZoomRelativeToFit: number;
  wheelZoomSensitivity: number;
};

export type AppConfig = {
  controller: ControllerConfig;
  ui: UiConfig;
};

export type ViewportConfig = Pick<UiConfig, 'minZoom' | 'maxZoom' | 'minZoomRelativeToFit' | 'wheelZoomSensitivity'>;

export const DEFAULT_UI_CONFIG: UiConfig = {
  replanMaxSpeedKmh: 5,
  maxGlobalPlannerDisplayBatches: 32,
  minZoom: 0.05,
  maxZoom: 100,
  minZoomRelativeToFit: 0.1,
  wheelZoomSensitivity: 0.0015,
};

type NumericField = {
  path: readonly string[];
  integer?: boolean;
  min?: number;
};

const NUMERIC_FIELDS: NumericField[] = [
  { path: ['controller', 'carConfig', 'wheelBase'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'length'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'width'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'backToWheel'], min: 0 },
  { path: ['controller', 'carConfig', 'wheelLength'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'wheelWidth'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'wheelSpacing'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'collisionPadding'], min: 0 },
  { path: ['controller', 'carConfig', 'targetMaxSteer'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'maxSteer'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'maxSteerSpeed'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'maxSpeed'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'minSpeed'] },
  { path: ['controller', 'carConfig', 'maxAccel'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'maxCentripetalAccel'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'carConfig', 'targetSpeed'] },
  { path: ['controller', 'carConfig', 'scanRadius'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'hybridAStarConfig', 'xyGridResolution'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'hybridAStarConfig', 'yawGridResolution'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'hybridAStarConfig', 'motionDistance'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'hybridAStarConfig', 'motionResolution'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'hybridAStarConfig', 'numSteerCommands'], integer: true, min: 1 },
  { path: ['controller', 'hybridAStarConfig', 'reedsSheppMaxDistance'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'switchDirectionCost'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'backwardsCost'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'steerChangeCost'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'steerCost'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'heuristicDistanceCost'], min: 0 },
  { path: ['controller', 'hybridAStarConfig', 'heuristicYawCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'horizonLength'], integer: true, min: 1 },
  { path: ['controller', 'mpcConfig', 'maxIterations'], integer: true, min: 1 },
  { path: ['controller', 'mpcConfig', 'duThreshold'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'mpcConfig', 'accelCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'steerCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'accelDeltaCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'steerDeltaCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'xCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'yCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'velocityCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'yawCost'], min: 0 },
  { path: ['controller', 'mpcConfig', 'terminalCostScale'], min: 0 },
  { path: ['controller', 'mpcConfig', 'desiredMaxAccelRatio'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'mpcConfig', 'minHorizonDistance'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'mpcConfig', 'directionChangeDistance'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'mpcConfig', 'motionResolution'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'runtime', 'simulationDeltaTime'], min: MIN_RANGE_VALUE },
  { path: ['controller', 'runtime', 'simulationIntervalMs'], integer: true, min: 1 },
  { path: ['controller', 'runtime', 'simulationPublishIntervalMs'], integer: true, min: 1 },
  { path: ['controller', 'runtime', 'localPlannerUpdateIntervalMs'], integer: true, min: 1 },
  { path: ['controller', 'runtime', 'hybridAStarStepBudget'], integer: true, min: 1 },
  { path: ['controller', 'runtime', 'hybridAStarSegmentBatchSize'], integer: true, min: 1 },
  { path: ['controller', 'runtime', 'mpcTimeStep'], min: MIN_RANGE_VALUE },
  { path: ['ui', 'replanMaxSpeedKmh'], min: 0 },
  { path: ['ui', 'maxGlobalPlannerDisplayBatches'], integer: true, min: 1 },
  { path: ['ui', 'minZoom'], min: MIN_RANGE_VALUE },
  { path: ['ui', 'maxZoom'], min: MIN_RANGE_VALUE },
  { path: ['ui', 'minZoomRelativeToFit'], min: 0 },
  { path: ['ui', 'wheelZoomSensitivity'], min: MIN_RANGE_VALUE },
];

function readNestedValue(source: unknown, path: readonly string[]) {
  return path.reduce<unknown>((current, key) => {
    if (!current || typeof current !== 'object' || !(key in current)) {
      return undefined;
    }
    return (current as Record<string, unknown>)[key];
  }, source);
}

function findWritableParent(target: AppConfig, path: readonly string[]) {
  const [firstKey, ...restPath] = path;
  let current: Record<string, unknown> = target[firstKey as keyof AppConfig] as unknown as Record<string, unknown>;
  for (let index = 0; index < restPath.length - 1; index += 1) {
    current = current[restPath[index]] as Record<string, unknown>;
  }
  return {
    parent: current,
    key: restPath[restPath.length - 1],
  };
}

function writeNestedValue(target: AppConfig, path: readonly string[], value: number) {
  const { parent, key } = findWritableParent(target, path);
  parent[key] = value;
}

function normalizeValue(field: NumericField, value: number) {
  const nextValue = field.integer ? Math.round(value) : value;
  return field.min === undefined ? nextValue : Math.max(field.min, nextValue);
}

export function createDefaultAppConfig(): AppConfig {
  return {
    controller: getDefaultControllerConfig(),
    ui: { ...DEFAULT_UI_CONFIG },
  };
}

export function cloneAppConfig(config: AppConfig): AppConfig {
  return {
    controller: cloneControllerConfig(config.controller),
    ui: { ...config.ui },
  };
}

export function getNumericAppConfigValue(config: AppConfig, path: readonly string[]) {
  const value = readNestedValue(config, path);
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new Error(`Invalid numeric settings path: ${path.join('.')}`);
  }
  return value;
}

export function updateNumericAppConfigValue(config: AppConfig, path: readonly string[], value: number): AppConfig {
  const nextConfig = cloneAppConfig(config);
  const { parent, key } = findWritableParent(nextConfig, path);
  parent[key] = value;
  return nextConfig;
}

/**
 * Normalizes persisted settings and also enforces cross-field constraints that
 * individual field validation cannot express:
 * - targetMaxSteer cannot exceed maxSteer
 * - targetSpeed is clamped between minSpeed and maxSpeed
 * - maxZoom cannot be smaller than minZoom
 */
export function sanitizeAppConfig(source: unknown): AppConfig {
  const sanitized = createDefaultAppConfig();
  for (const field of NUMERIC_FIELDS) {
    const rawValue = readNestedValue(source, field.path);
    if (typeof rawValue !== 'number' || !Number.isFinite(rawValue)) {
      continue;
    }
    writeNestedValue(sanitized, field.path, normalizeValue(field, rawValue));
  }

  const { carConfig } = sanitized.controller;
  carConfig.targetMaxSteer = Math.min(carConfig.targetMaxSteer, carConfig.maxSteer);
  carConfig.targetSpeed = Math.min(Math.max(carConfig.targetSpeed, carConfig.minSpeed), carConfig.maxSpeed);
  sanitized.ui.maxZoom = Math.max(sanitized.ui.maxZoom, sanitized.ui.minZoom);

  return sanitized;
}

function canUseLocalStorage() {
  if (typeof window === 'undefined' || typeof window.localStorage === 'undefined') {
    return false;
  }
  try {
    const probeKey = `${APP_CONFIG_STORAGE_KEY}:probe`;
    window.localStorage.setItem(probeKey, probeKey);
    window.localStorage.removeItem(probeKey);
    return true;
  } catch {
    return false;
  }
}

export function loadStoredAppConfig(): AppConfig {
  if (!canUseLocalStorage()) {
    return createDefaultAppConfig();
  }
  try {
    const rawValue = window.localStorage.getItem(APP_CONFIG_STORAGE_KEY);
    return rawValue ? sanitizeAppConfig(JSON.parse(rawValue)) : createDefaultAppConfig();
  } catch (error) {
    console.warn('Failed to load stored app settings', error);
    return createDefaultAppConfig();
  }
}

export function persistAppConfig(config: AppConfig): AppConfig {
  const sanitized = sanitizeAppConfig(config);
  if (!canUseLocalStorage()) {
    return sanitized;
  }
  try {
    window.localStorage.setItem(APP_CONFIG_STORAGE_KEY, JSON.stringify(sanitized));
  } catch (error) {
    console.warn('Failed to persist app settings', error);
  }
  return sanitized;
}

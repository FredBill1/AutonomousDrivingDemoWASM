import { DEG_TO_RAD, KMH_TO_MS } from './constants';
import type {
  CarConfigSnapshot,
  ControllerConfig,
  HybridAStarConfigSnapshot,
  MpcConfigSnapshot,
} from './workerContracts';

export const DEFAULT_CAR_CONFIG: CarConfigSnapshot = {
  wheelBase: 2.5,
  length: 4.5,
  width: 2.0,
  backToWheel: 1.0,
  wheelLength: 0.8,
  wheelWidth: 0.5,
  wheelSpacing: 1.4,
  collisionPadding: 0.5,
  targetMaxSteer: 35 * DEG_TO_RAD,
  maxSteer: 40 * DEG_TO_RAD,
  maxSteerSpeed: 360 * DEG_TO_RAD,
  maxSpeed: 55 * KMH_TO_MS,
  minSpeed: -30 * KMH_TO_MS,
  maxAccel: 15,
  maxCentripetalAccel: 16,
  targetSpeed: 40 * KMH_TO_MS,
  scanRadius: 15,
};

export const DEFAULT_HYBRID_ASTAR_CONFIG: HybridAStarConfigSnapshot = {
  xyGridResolution: 1,
  yawGridResolution: 15 * DEG_TO_RAD,
  motionDistance: 1.5,
  motionResolution: 0.5,
  numSteerCommands: 10,
  reedsSheppMaxDistance: 10,
  switchDirectionCost: 25,
  backwardsCost: 4,
  steerChangeCost: 3,
  steerCost: 1.5,
  heuristicDistanceCost: 2,
  heuristicYawCost: 3 / (45 * DEG_TO_RAD),
};

export const DEFAULT_MPC_CONFIG: MpcConfigSnapshot = {
  horizonLength: 5,
  maxIterations: 5,
  duThreshold: 0.1,
  accelCost: 0.01,
  steerCost: 0.005,
  accelDeltaCost: 1e-5,
  steerDeltaCost: 1e-3,
  xCost: 1.1,
  yCost: 1.1,
  velocityCost: 0.05,
  yawCost: 1.1,
  terminalCostScale: 2,
  desiredMaxAccelRatio: 0.7,
  minHorizonDistance: 0.3,
  directionChangeDistance: 0.1,
  motionResolution: 0.5,
};

export const DEFAULT_CONTROLLER_CONFIG: ControllerConfig = {
  carConfig: DEFAULT_CAR_CONFIG,
  hybridAStarConfig: DEFAULT_HYBRID_ASTAR_CONFIG,
  mpcConfig: DEFAULT_MPC_CONFIG,
  runtime: {
    simulationDeltaTime: 0.015,
    simulationIntervalMs: 20,
    simulationPublishIntervalMs: 50,
    localPlannerUpdateIntervalMs: 100,
    hybridAStarStepBudget: 96,
    hybridAStarSegmentBatchSize: 320,
    mpcTimeStep: 0.07,
  },
};

export function getDefaultControllerConfig(): ControllerConfig {
  return DEFAULT_CONTROLLER_CONFIG;
}

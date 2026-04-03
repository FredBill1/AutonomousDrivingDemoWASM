import { CarConfig, HybridAStarConfig, MpcConfig } from '../../wasm-core/pkg/wasm_core';

import type {
  CarConfigSnapshot,
  ControllerConfig,
  HybridAStarConfigSnapshot,
  MpcConfigSnapshot,
  WasmConfigSnapshot,
  WasmRuntime,
} from './workerContracts';

export function toWasmConfigSnapshot(config: CarConfigSnapshot): WasmConfigSnapshot {
  const backToCenter = config.length / 2 - config.backToWheel;
  const collisionLength = config.length + config.collisionPadding;
  const collisionWidth = config.width + config.collisionPadding;
  return {
    ...config,
    backToCenter,
    collisionLength,
    collisionWidth,
    collisionRadius: Math.hypot(collisionWidth / 2, collisionLength / 2),
    targetMinTurningRadius: config.wheelBase / Math.tan(config.targetMaxSteer),
  };
}

export function createCarConfig(config: CarConfigSnapshot) {
  return new CarConfig(
    config.wheelBase,
    config.length,
    config.width,
    config.backToWheel,
    config.collisionPadding,
    config.targetMaxSteer,
    config.maxSteer,
    config.maxSteerSpeed,
    config.maxSpeed,
    config.minSpeed,
    config.maxAccel,
    config.maxCentripetalAccel,
    config.targetSpeed,
    config.scanRadius,
  );
}

export function createHybridAStarConfig(config: HybridAStarConfigSnapshot) {
  return new HybridAStarConfig(
    config.xyGridResolution,
    config.yawGridResolution,
    config.motionDistance,
    config.motionResolution,
    config.numSteerCommands,
    config.reedsSheppMaxDistance,
    config.switchDirectionCost,
    config.backwardsCost,
    config.steerChangeCost,
    config.steerCost,
    config.heuristicDistanceCost,
    config.heuristicYawCost,
  );
}

export function createMpcConfig(config: MpcConfigSnapshot) {
  return new MpcConfig(
    config.horizonLength,
    config.maxIterations,
    config.duThreshold,
    config.accelCost,
    config.steerCost,
    config.accelDeltaCost,
    config.steerDeltaCost,
    config.xCost,
    config.yCost,
    config.velocityCost,
    config.yawCost,
    config.terminalCostScale,
    config.desiredMaxAccelRatio,
    config.minHorizonDistance,
    config.directionChangeDistance,
    config.motionResolution,
  );
}

export function createWasmRuntime(controllerConfig: ControllerConfig): WasmRuntime {
  return {
    carConfig: createCarConfig(controllerConfig.carConfig),
    hybridAStarConfig: createHybridAStarConfig(controllerConfig.hybridAStarConfig),
    mpcConfig: createMpcConfig(controllerConfig.mpcConfig),
  };
}

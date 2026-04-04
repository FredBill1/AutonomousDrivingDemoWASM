import { cloneAppConfig, type AppConfig } from './appConfig';
import { DEG_TO_RAD, KMH_TO_MS } from './constants';

export type SettingsFieldDefinition = {
  path: readonly string[];
  label: string;
  description: string;
  step: number;
  min?: number;
};

export type SettingsSectionDefinition = {
  title: string;
  description: string;
  fields: SettingsFieldDefinition[];
};

function field(path: readonly string[], label: string, description: string, step: number, min?: number) {
  return { path, label, description, step, min } satisfies SettingsFieldDefinition;
}

export const SETTINGS_SECTIONS: SettingsSectionDefinition[] = [
  {
    title: 'Vehicle geometry',
    description: 'Vehicle dimensions, wheel layout, and collision shape parameters.',
    fields: [
      field(['controller', 'carConfig', 'wheelBase'], 'Wheel base', 'Distance between rear and front axles.', 0.1, 0.001),
      field(['controller', 'carConfig', 'length'], 'Body length', 'Vehicle body length used for drawing and collision.', 0.1, 0.001),
      field(['controller', 'carConfig', 'width'], 'Body width', 'Vehicle body width used for drawing and collision.', 0.1, 0.001),
      field(['controller', 'carConfig', 'backToWheel'], 'Rear overhang', 'Distance from rear axle to vehicle tail.', 0.1, 0),
      field(['controller', 'carConfig', 'wheelLength'], 'Wheel length', 'Visualized wheel rectangle length.', 0.05, 0.001),
      field(['controller', 'carConfig', 'wheelWidth'], 'Wheel width', 'Visualized wheel rectangle width.', 0.05, 0.001),
      field(['controller', 'carConfig', 'wheelSpacing'], 'Wheel spacing', 'Track width used for the wheel visuals.', 0.05, 0.001),
      field(['controller', 'carConfig', 'collisionPadding'], 'Collision padding', 'Extra safety padding around the vehicle body.', 0.05, 0),
      field(['controller', 'carConfig', 'scanRadius'], 'Scan radius', 'Obstacle sensing radius around the vehicle.', 0.5, 0.001),
    ],
  },
  {
    title: 'Vehicle motion',
    description: 'Steering, speed, and acceleration limits for the car model.',
    fields: [
      field(['controller', 'carConfig', 'targetMaxSteer'], 'Target max steer', 'Target steering angle limit.', 1 * DEG_TO_RAD, 0.001),
      field(['controller', 'carConfig', 'maxSteer'], 'Max steer', 'Absolute steering angle limit.', 1 * DEG_TO_RAD, 0.001),
      field(['controller', 'carConfig', 'maxSteerSpeed'], 'Max steer speed', 'Maximum steering rate.', 5 * DEG_TO_RAD, 0.001),
      field(['controller', 'carConfig', 'maxSpeed'], 'Max speed', 'Maximum forward speed.', 1 * KMH_TO_MS, 0.001),
      field(['controller', 'carConfig', 'minSpeed'], 'Min speed', 'Maximum reverse speed.', 1 * KMH_TO_MS),
      field(['controller', 'carConfig', 'maxAccel'], 'Max acceleration', 'Maximum longitudinal acceleration.', 0.5, 0.001),
      field(
        ['controller', 'carConfig', 'maxCentripetalAccel'],
        'Max centripetal accel',
        'Maximum turning acceleration before clipping.',
        0.5,
        0.001,
      ),
      field(['controller', 'carConfig', 'targetSpeed'], 'Target speed', 'Cruising speed used by the planner.', 1 * KMH_TO_MS),
    ],
  },
  {
    title: 'Hybrid A* planner',
    description: 'Search discretization and path cost tuning for the global planner.',
    fields: [
      field(['controller', 'hybridAStarConfig', 'xyGridResolution'], 'XY grid resolution', 'Cell size of the planner grid.', 0.1, 0.001),
      field(
        ['controller', 'hybridAStarConfig', 'yawGridResolution'],
        'Yaw grid resolution',
        'Heading discretization of the planner grid.',
        1 * DEG_TO_RAD,
        0.001,
      ),
      field(['controller', 'hybridAStarConfig', 'motionDistance'], 'Motion distance', 'Expansion length for each search step.', 0.1, 0.001),
      field(
        ['controller', 'hybridAStarConfig', 'motionResolution'],
        'Motion resolution',
        'Sampling resolution along each search step.',
        0.05,
        0.001,
      ),
      field(['controller', 'hybridAStarConfig', 'numSteerCommands'], 'Steer commands', 'Number of steering choices per expansion.', 1, 1),
      field(
        ['controller', 'hybridAStarConfig', 'reedsSheppMaxDistance'],
        'Reeds-Shepp max distance',
        'Maximum analytic expansion distance.',
        0.5,
        0,
      ),
      field(
        ['controller', 'hybridAStarConfig', 'switchDirectionCost'],
        'Direction switch cost',
        'Penalty for forward/reverse transitions.',
        0.5,
        0,
      ),
      field(['controller', 'hybridAStarConfig', 'backwardsCost'], 'Backwards cost', 'Penalty for reverse motion.', 0.25, 0),
      field(
        ['controller', 'hybridAStarConfig', 'steerChangeCost'],
        'Steer change cost',
        'Penalty for changing steering between steps.',
        0.25,
        0,
      ),
      field(['controller', 'hybridAStarConfig', 'steerCost'], 'Steer cost', 'Penalty for steering magnitude.', 0.25, 0),
      field(
        ['controller', 'hybridAStarConfig', 'heuristicDistanceCost'],
        'Heuristic distance cost',
        'Distance term weight in the heuristic.',
        0.25,
        0,
      ),
      field(['controller', 'hybridAStarConfig', 'heuristicYawCost'], 'Heuristic yaw cost', 'Heading term weight in the heuristic.', 0.25, 0),
    ],
  },
  {
    title: 'MPC controller',
    description: 'Preview horizon, convergence controls, and tracking cost weights.',
    fields: [
      field(['controller', 'mpcConfig', 'horizonLength'], 'Horizon length', 'Number of MPC steps in the preview horizon.', 1, 1),
      field(['controller', 'mpcConfig', 'maxIterations'], 'Max iterations', 'Maximum solver iterations per update.', 1, 1),
      field(['controller', 'mpcConfig', 'duThreshold'], 'DU threshold', 'Convergence threshold for control updates.', 0.01, 0.001),
      field(['controller', 'mpcConfig', 'accelCost'], 'Acceleration cost', 'Weight on acceleration magnitude.', 0.001, 0),
      field(['controller', 'mpcConfig', 'steerCost'], 'Steer cost', 'Weight on steering magnitude.', 0.001, 0),
      field(['controller', 'mpcConfig', 'accelDeltaCost'], 'Acceleration delta cost', 'Weight on acceleration smoothness.', 0.00001, 0),
      field(['controller', 'mpcConfig', 'steerDeltaCost'], 'Steer delta cost', 'Weight on steering smoothness.', 0.0001, 0),
      field(['controller', 'mpcConfig', 'xCost'], 'X cost', 'Weight on longitudinal tracking error.', 0.1, 0),
      field(['controller', 'mpcConfig', 'yCost'], 'Y cost', 'Weight on lateral tracking error.', 0.1, 0),
      field(['controller', 'mpcConfig', 'velocityCost'], 'Velocity cost', 'Weight on speed tracking error.', 0.01, 0),
      field(['controller', 'mpcConfig', 'yawCost'], 'Yaw cost', 'Weight on heading tracking error.', 0.1, 0),
      field(['controller', 'mpcConfig', 'terminalCostScale'], 'Terminal cost scale', 'Multiplier applied to the terminal state cost.', 0.1, 0),
      field(
        ['controller', 'mpcConfig', 'desiredMaxAccelRatio'],
        'Desired max accel ratio',
        'Requested ratio of the configured max acceleration.',
        0.05,
        0.001,
      ),
      field(
        ['controller', 'mpcConfig', 'minHorizonDistance'],
        'Min horizon distance',
        'Minimum forward distance covered by the MPC horizon.',
        0.05,
        0.001,
      ),
      field(
        ['controller', 'mpcConfig', 'directionChangeDistance'],
        'Direction change distance',
        'Distance threshold for path direction changes.',
        0.05,
        0.001,
      ),
      field(['controller', 'mpcConfig', 'motionResolution'], 'Motion resolution', 'Reference sampling resolution for MPC.', 0.05, 0.001),
    ],
  },
  {
    title: 'Runtime and workers',
    description: 'Worker update rates and timing values shared with the simulation runtime.',
    fields: [
      field(['controller', 'runtime', 'simulationDeltaTime'], 'Simulation delta time', 'Physics integration time step.', 0.001, 0.001),
      field(['controller', 'runtime', 'simulationIntervalMs'], 'Simulation interval', 'Worker loop interval for simulation updates.', 1, 1),
      field(
        ['controller', 'runtime', 'simulationPublishIntervalMs'],
        'Simulation publish interval',
        'How often simulation state is published to the UI.',
        1,
        1,
      ),
      field(
        ['controller', 'runtime', 'localPlannerUpdateIntervalMs'],
        'Local planner interval',
        'How often the local planner worker recomputes controls.',
        1,
        1,
      ),
      field(
        ['controller', 'runtime', 'hybridAStarStepBudget'],
        'Hybrid A* step budget',
        'Number of search steps processed between yielding back to the worker loop.',
        1,
        1,
      ),
      field(
        ['controller', 'runtime', 'hybridAStarSegmentBatchSize'],
        'Hybrid A* segment batch size',
        'Global planner progress segments sent per UI batch.',
        1,
        1,
      ),
      field(['controller', 'runtime', 'mpcTimeStep'], 'MPC time step', 'Time step used by the MPC controller.', 0.001, 0.001),
    ],
  },
  {
    title: 'UI and viewport',
    description: 'Planning display limits and map interaction behavior stored in the browser.',
    fields: [
      field(['ui', 'replanMaxSpeedKmh'], 'Replan max speed', 'Maximum speed that still replans from the current vehicle state.', 0.5, 0),
      field(
        ['ui', 'maxGlobalPlannerDisplayBatches'],
        'Planner display batches',
        'Maximum number of global planner progress batches kept in the UI.',
        1,
        1,
      ),
      field(['ui', 'minZoom'], 'Minimum zoom', 'Lower zoom clamp for viewport interactions.', 0.01, 0.001),
      field(['ui', 'maxZoom'], 'Maximum zoom', 'Upper zoom clamp for viewport interactions.', 0.1, 0.001),
      field(
        ['ui', 'minZoomRelativeToFit'],
        'Min zoom relative to fit',
        'Lower zoom clamp relative to the fitted map scale.',
        0.01,
        0,
      ),
      field(
        ['ui', 'wheelZoomSensitivity'],
        'Wheel zoom sensitivity',
        'Mouse wheel zoom response multiplier.',
        0.0001,
        0.001,
      ),
    ],
  },
];

export function getNumericSettingValue(config: AppConfig, path: readonly string[]) {
  return path.reduce<unknown>((current, key) => (current as Record<string, unknown>)[key], config) as number;
}

export function updateNumericSettingValue(config: AppConfig, path: readonly string[], value: number): AppConfig {
  const nextConfig = cloneAppConfig(config);

  const [firstKey, ...restPath] = path;
  let current: Record<string, unknown> = nextConfig[firstKey as keyof AppConfig] as unknown as Record<string, unknown>;
  for (let index = 0; index < restPath.length - 1; index += 1) {
    current = current[restPath[index]] as Record<string, unknown>;
  }
  current[restPath[restPath.length - 1]] = value;
  return nextConfig;
}

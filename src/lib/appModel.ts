export type Mode = 'goal' | 'pose';

export type CarState = {
  x: number;
  y: number;
  yaw: number;
  velocity: number;
  steer: number;
};

export type Obstacle = {
  id: string;
  x: number;
  y: number;
};

export type PathPoint = {
  x: number;
  y: number;
  yaw: number;
};

export const HISTORY_LIMIT = 500;
export const CAR_CONSTANTS = {
  wheelBase: 2.5,
  length: 4.5,
  width: 2,
  backToWheel: 1,
  wheelLength: 0.8,
  wheelWidth: 0.5,
  wheelSpacing: 1.4,
  backToCenter: 1.25,
  collisionLength: 5,
  collisionWidth: 2.5,
  collisionRadius: Math.hypot(2.5, 1.25),
  maxSteer: (40 * Math.PI) / 180,
  targetMaxSteer: (35 * Math.PI) / 180,
  maxSteerSpeed: (360 * Math.PI) / 180,
  maxSpeed: 55 / 3.6,
  minSpeed: -30 / 3.6,
  maxAccel: 15,
  maxCentripetalAccel: 16,
  targetSpeed: 40 / 3.6,
  targetMinTurningRadius: 2.5 / Math.tan((35 * Math.PI) / 180),
  scanRadius: 15,
} as const;

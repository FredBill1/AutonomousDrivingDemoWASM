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

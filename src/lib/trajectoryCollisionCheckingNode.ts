import type { PathPoint } from './appModel';

export const TRAJECTORY_COLLISION_DISCARD_FIRST_N = 5;

type CollisionCheckFn = (path: PathPoint[], obstacleCoordinates: Float64Array) => Promise<boolean>;

export class TrajectoryCollisionCheckingNode {
  private trajectory: PathPoint[] | null = null;
  private knownObstacleCoordinates: Float64Array | null = null;
  private generation = 0;
  private collidedListener: (() => void) | null = null;

  constructor(private readonly checkTrajectoryCollision: CollisionCheckFn) {}

  setCollidedListener(listener: (() => void) | null) {
    this.collidedListener = listener;
  }

  setTrajectory(trajectory: PathPoint[] | null): Promise<boolean> {
    this.generation += 1;
    const generation = this.generation;
    this.trajectory =
      trajectory === null
        ? null
        : trajectory.slice(TRAJECTORY_COLLISION_DISCARD_FIRST_N).map((point) => ({
            x: point.x,
            y: point.y,
            yaw: point.yaw,
          }));

    const knownObstacleCoordinates = this.knownObstacleCoordinates;
    if (this.trajectory === null || !knownObstacleCoordinates) {
      return Promise.resolve(false);
    }

    return this.checkCurrentTrajectory(this.trajectory, knownObstacleCoordinates, generation);
  }

  setKnownObstacles(knownObstacleCoordinates: Float64Array) {
    this.knownObstacleCoordinates = Float64Array.from(knownObstacleCoordinates);
  }

  checkCollision(obstacleCoordinates: Float64Array): Promise<boolean> {
    const trajectory = this.trajectory;
    if (trajectory === null) {
      return Promise.resolve(false);
    }

    return this.checkCurrentTrajectory(trajectory, obstacleCoordinates, this.generation);
  }

  cancel() {
    this.generation += 1;
    this.trajectory = null;
  }

  private async checkCurrentTrajectory(trajectory: PathPoint[], obstacleCoordinates: Float64Array, generation: number) {
    const collided = await this.checkTrajectoryCollision(trajectory, obstacleCoordinates);
    if (!collided) {
      return false;
    }

    if (this.generation !== generation || this.trajectory !== trajectory) {
      return false;
    }

    this.collidedListener?.();
    return true;
  }
}

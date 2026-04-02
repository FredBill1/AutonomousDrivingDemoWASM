import type { CarConfig, HybridAStarPlanner, MpcConfig, MpcReferenceTracker } from '../../wasm-core/pkg/wasm_core';

export type WasmPose = {
  x: number;
  y: number;
  yaw: number;
};

export type WasmCarState = WasmPose & {
  velocity: number;
  steer: number;
};

export type HybridAStarProgressSegment = {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
};

export type LocalPlannerPathPoint = WasmPose;

export type LocalPlannerReferencePoint = WasmPose & {
  velocity: number;
};

export type LocalPlannerTrajectoryPoint = WasmPose & {
  direction: number;
};

export type LocalPlannerControlPoint = {
  timestamp: number;
  targetVelocity: number;
  targetSteer: number;
};

export type HybridAStarStartSeedPoint = WasmPose & {
  velocity: number;
};

export type HybridAStarSolution = {
  token: number;
  path: WasmPose[];
  directions: number[];
  exploredSegments: HybridAStarProgressSegment[];
  exploredCount: number;
  analyticExpansions: number;
};

export type HybridAStarProgress = {
  token: number;
  segments: HybridAStarProgressSegment[];
  exploredCount: number;
  analyticExpansions: number;
};

export type LocalPlannerUpdateResult = {
  controlSequence: LocalPlannerControlPoint[];
  localTrajectory: LocalPlannerPathPoint[];
  referencePoints: LocalPlannerReferencePoint[];
  brakeTrajectory: LocalPlannerReferencePoint[];
};

export type TrackingPlan = {
  path: WasmPose[];
  directions: number[];
};

export type WasmConfigSnapshot = {
  wheelBase: number;
  length: number;
  width: number;
  backToWheel: number;
  wheelLength: number;
  wheelWidth: number;
  wheelSpacing: number;
  backToCenter: number;
  collisionLength: number;
  collisionWidth: number;
  collisionRadius: number;
  targetMaxSteer: number;
  maxSteer: number;
  maxSteerSpeed: number;
  maxSpeed: number;
  minSpeed: number;
  maxAccel: number;
  maxCentripetalAccel: number;
  targetSpeed: number;
  targetMinTurningRadius: number;
  scanRadius: number;
};

export type SimulationStateEvent = {
  timestamp: number;
  state: WasmCarState;
};

export type WorkerMethodSpec<Payload, Result> = {
  payload: Payload;
  result: Result;
};

export type WorkerMethodMap = {
  getCarConfigSnapshot: WorkerMethodSpec<undefined, WasmConfigSnapshot>;
  stepCarState: WorkerMethodSpec<
    { current: WasmCarState; targetVelocity: number; targetSteer: number; dt: number },
    WasmCarState
  >;
  initSimulation: WorkerMethodSpec<
    {
      state: WasmCarState;
      timestamp?: number;
      simDeltaTime?: number;
      simulationIntervalMs?: number;
      publishIntervalMs?: number;
    },
    null
  >;
  setSimulationState: WorkerMethodSpec<{ state: WasmCarState; timestamp?: number }, null>;
  setSimulationControlSequence: WorkerMethodSpec<{ controlSequence: LocalPlannerControlPoint[] }, null>;
  stopSimulationMotion: WorkerMethodSpec<undefined, null>;
  resumeSimulationMotion: WorkerMethodSpec<undefined, null>;
  stopSimulation: WorkerMethodSpec<undefined, null>;
  checkCollision: WorkerMethodSpec<{ state: WasmCarState; obstacleCoordinates: number[] }, boolean>;
  checkPathCollision: WorkerMethodSpec<{ path: WasmPose[]; obstacleCoordinates: number[] }, boolean>;
  checkTrajectoryCollision: WorkerMethodSpec<{ path: WasmPose[]; obstacleCoordinates: number[] }, boolean>;
  solveHybridAStar: WorkerMethodSpec<
    {
      start: WasmCarState | HybridAStarStartSeedPoint[];
      startIsTrajectorySeed: boolean;
      goal: WasmCarState;
      obstacleCoordinates: number[];
      maxIterations: number;
      requestToken?: number;
    },
    HybridAStarSolution | null
  >;
  cancelHybridAStar: WorkerMethodSpec<undefined, null>;
  setLocalPlannerTrajectory: WorkerMethodSpec<{ trajectory: LocalPlannerTrajectoryPoint[] | null }, null>;
  setLocalPlannerState: WorkerMethodSpec<{ state: WasmCarState; timestamp: number; updateIntervalMs?: number }, null>;
  brakeLocalPlanner: WorkerMethodSpec<undefined, null>;
  cancelLocalPlanner: WorkerMethodSpec<undefined, null>;
  solveReedsSheppCandidates: WorkerMethodSpec<
    {
      start: WasmCarState;
      goal: WasmCarState;
      turnRadii: number[];
      runwayLengths: number[];
      stepSize: number;
      lengthTolerance: number;
    },
    Array<{
      path: WasmPose[];
      totalLength: number;
      segmentCount: number;
      runwayLength: number;
      turnRadius: number;
    }>
  >;
};

export type WorkerEventMap = {
  hybridAStarProgress: HybridAStarProgress;
  simulationState: SimulationStateEvent;
  localPlannerUpdate: LocalPlannerUpdateResult;
};

export type WorkerRequest = {
  [Key in keyof WorkerMethodMap]: undefined extends WorkerMethodMap[Key]['payload']
    ? { id: number; type: Key; payload?: WorkerMethodMap[Key]['payload'] }
    : { id: number; type: Key; payload: WorkerMethodMap[Key]['payload'] };
}[keyof WorkerMethodMap];

export type WorkerResponse = {
  [Key in keyof WorkerMethodMap]:
    | { id: number; ok: true; result: WorkerMethodMap[Key]['result'] }
    | { id: number; ok: false; error: string };
}[keyof WorkerMethodMap];

export type WorkerEvent = {
  [Key in keyof WorkerEventMap]: { type: Key; payload: WorkerEventMap[Key] };
}[keyof WorkerEventMap];

export type WorkerHandlerMap = {
  [Key in keyof WorkerMethodMap]: (payload: WorkerMethodMap[Key]['payload']) => Promise<WorkerMethodMap[Key]['result']>;
};

export type WasmRuntime = {
  carConfig: CarConfig;
  mpcConfig: MpcConfig;
};

export type PlannerSession = {
  planner: HybridAStarPlanner;
  cancelled: boolean;
};

export type SimulationSession = {
  state: WasmCarState;
  timestamp: number;
  simDeltaTime: number;
  controlSequence: LocalPlannerControlPoint[] | null;
  stopped: boolean;
  simulationTimerId: ReturnType<typeof setTimeout> | null;
  publishTimerId: ReturnType<typeof setInterval> | null;
  loopToken: number;
  stateVersion: number;
};

export type LocalPlannerSession = {
  tracker: MpcReferenceTracker | null;
  latestState: { state: WasmCarState; timestamp: number } | null;
  updateIntervalMs: number;
  updateTimerId: ReturnType<typeof setInterval> | null;
  updateInFlight: boolean;
};

export const HYBRID_SEGMENT_BATCH_SIZE = 320;
export const DEFAULT_SIM_DELTA_TIME = 0.015;
export const DEFAULT_SIM_INTERVAL_MS = 20;
export const DEFAULT_PUBLISH_INTERVAL_MS = 50;
export const DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS = 100;
export const HYBRID_ASTAR_STEP_BUDGET = 96;
export const MPC_DT = 0.07;

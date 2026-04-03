import type { CarConfig, HybridAStarConfig, MpcConfig, MpcReferenceTracker } from '../../wasm-core/pkg/wasm_core';

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

export type CarConfigSnapshot = {
  wheelBase: number;
  length: number;
  width: number;
  backToWheel: number;
  wheelLength: number;
  wheelWidth: number;
  wheelSpacing: number;
  collisionPadding: number;
  targetMaxSteer: number;
  maxSteer: number;
  maxSteerSpeed: number;
  maxSpeed: number;
  minSpeed: number;
  maxAccel: number;
  maxCentripetalAccel: number;
  targetSpeed: number;
  scanRadius: number;
};

export type HybridAStarConfigSnapshot = {
  xyGridResolution: number;
  yawGridResolution: number;
  motionDistance: number;
  motionResolution: number;
  numSteerCommands: number;
  reedsSheppMaxDistance: number;
  switchDirectionCost: number;
  backwardsCost: number;
  steerChangeCost: number;
  steerCost: number;
  heuristicDistanceCost: number;
  heuristicYawCost: number;
};

export type MpcConfigSnapshot = {
  horizonLength: number;
  maxIterations: number;
  duThreshold: number;
  accelCost: number;
  steerCost: number;
  accelDeltaCost: number;
  steerDeltaCost: number;
  xCost: number;
  yCost: number;
  velocityCost: number;
  yawCost: number;
  terminalCostScale: number;
  desiredMaxAccelRatio: number;
  minHorizonDistance: number;
  directionChangeDistance: number;
  motionResolution: number;
};

export type RuntimeConfigSnapshot = {
  simulationDeltaTime: number;
  simulationIntervalMs: number;
  simulationPublishIntervalMs: number;
  localPlannerUpdateIntervalMs: number;
  hybridAStarStepBudget: number;
  hybridAStarSegmentBatchSize: number;
  mpcTimeStep: number;
};

export type ControllerConfig = {
  carConfig: CarConfigSnapshot;
  hybridAStarConfig: HybridAStarConfigSnapshot;
  mpcConfig: MpcConfigSnapshot;
  runtime: RuntimeConfigSnapshot;
};

export type WasmConfigSnapshot = CarConfigSnapshot & {
  backToCenter: number;
  collisionLength: number;
  collisionWidth: number;
  collisionRadius: number;
  targetMinTurningRadius: number;
};

export type SimulationStateEvent = {
  timestamp: number;
  state: WasmCarState;
};

export type WorkerMethodSpec<Payload, Result> = {
  payload: Payload;
  result: Result;
};

export type OrchestratorMethodMap = {
  initializeRuntime: WorkerMethodSpec<ControllerConfig, null>;
  getCarConfigSnapshot: WorkerMethodSpec<undefined, WasmConfigSnapshot>;
  stepCarState: WorkerMethodSpec<
    { current: WasmCarState; targetVelocity: number; targetSteer: number; dt: number },
    WasmCarState
  >;
  initSimulation: WorkerMethodSpec<{ state: WasmCarState; timestamp?: number }, null>;
  setSimulationState: WorkerMethodSpec<{ state: WasmCarState; timestamp?: number }, null>;
  stopSimulationMotion: WorkerMethodSpec<undefined, null>;
  resumeSimulationMotion: WorkerMethodSpec<undefined, null>;
  stopSimulation: WorkerMethodSpec<undefined, null>;
  checkCollision: WorkerMethodSpec<{ state: WasmCarState; obstacleCoordinates: Float64Array }, boolean>;
  checkPathCollision: WorkerMethodSpec<{ path: WasmPose[]; obstacleCoordinates: Float64Array }, boolean>;
  checkTrajectoryCollision: WorkerMethodSpec<{ path: WasmPose[]; obstacleCoordinates: Float64Array }, boolean>;
  solveHybridAStar: WorkerMethodSpec<
    {
      start: WasmCarState | HybridAStarStartSeedPoint[];
      startIsTrajectorySeed: boolean;
      goal: WasmCarState;
      obstacleCoordinates: Float64Array;
      maxIterations: number;
      requestToken?: number;
    },
    HybridAStarSolution | null
  >;
  cancelHybridAStar: WorkerMethodSpec<undefined, null>;
  setLocalPlannerTrajectory: WorkerMethodSpec<{ trajectory: LocalPlannerTrajectoryPoint[] | null }, null>;
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

export type OrchestratorEventMap = {
  hybridAStarProgress: HybridAStarProgress;
  simulationState: SimulationStateEvent;
  localPlannerUpdate: LocalPlannerUpdateResult;
};

export type SimulationWorkerMethodMap = {
  initializeRuntime: WorkerMethodSpec<{ carConfig: CarConfigSnapshot; runtime: RuntimeConfigSnapshot }, null>;
  stepCarState: OrchestratorMethodMap['stepCarState'];
  initSimulation: OrchestratorMethodMap['initSimulation'];
  setSimulationState: OrchestratorMethodMap['setSimulationState'];
  setSimulationControlSequence: WorkerMethodSpec<{ controlSequence: LocalPlannerControlPoint[] }, null>;
  stopSimulationMotion: OrchestratorMethodMap['stopSimulationMotion'];
  resumeSimulationMotion: OrchestratorMethodMap['resumeSimulationMotion'];
  stopSimulation: OrchestratorMethodMap['stopSimulation'];
};

export type SimulationWorkerEventMap = {
  simulationState: SimulationStateEvent;
};

export type LocalPlannerWorkerMethodMap = {
  initializeRuntime: WorkerMethodSpec<
    { carConfig: CarConfigSnapshot; mpcConfig: MpcConfigSnapshot; runtime: RuntimeConfigSnapshot },
    null
  >;
  setLocalPlannerTrajectory: OrchestratorMethodMap['setLocalPlannerTrajectory'];
  setLocalPlannerState: WorkerMethodSpec<{ state: WasmCarState; timestamp: number }, null>;
  brakeLocalPlanner: OrchestratorMethodMap['brakeLocalPlanner'];
  cancelLocalPlanner: OrchestratorMethodMap['cancelLocalPlanner'];
  stopLocalPlanner: WorkerMethodSpec<undefined, null>;
};

export type LocalPlannerWorkerEventMap = {
  localPlannerUpdate: LocalPlannerUpdateResult;
};

export type GlobalPlannerWorkerMethodMap = {
  initializeRuntime: WorkerMethodSpec<
    { carConfig: CarConfigSnapshot; hybridAStarConfig: HybridAStarConfigSnapshot; runtime: RuntimeConfigSnapshot },
    null
  >;
  solveHybridAStar: OrchestratorMethodMap['solveHybridAStar'];
  cancelHybridAStar: OrchestratorMethodMap['cancelHybridAStar'];
  solveReedsSheppCandidates: OrchestratorMethodMap['solveReedsSheppCandidates'];
};

export type GlobalPlannerWorkerEventMap = {
  hybridAStarProgress: HybridAStarProgress;
};

export type WorkerRequest<Methods extends Record<string, WorkerMethodSpec<unknown, unknown>>> = {
  [Key in keyof Methods]: undefined extends Methods[Key]['payload']
    ? { id: number; type: Key; payload?: Methods[Key]['payload'] }
    : { id: number; type: Key; payload: Methods[Key]['payload'] };
}[keyof Methods];

export type WorkerResponse<Methods extends Record<string, WorkerMethodSpec<unknown, unknown>>> = {
  [Key in keyof Methods]:
    | { id: number; ok: true; result: Methods[Key]['result'] }
    | { id: number; ok: false; error: string };
}[keyof Methods];

export type WorkerEvent<Events extends Record<string, unknown>> = {
  [Key in keyof Events]: { type: Key; payload: Events[Key] };
}[keyof Events];

export type WorkerHandlerMap<Methods extends Record<string, WorkerMethodSpec<unknown, unknown>>> = {
  [Key in keyof Methods]: (payload: Methods[Key]['payload']) => Promise<Methods[Key]['result']>;
};

export type WasmRuntime = {
  carConfig: CarConfig;
  hybridAStarConfig: HybridAStarConfig;
  mpcConfig: MpcConfig;
};

export type SimulationSession = {
  state: WasmCarState;
  timestamp: number;
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
  updateTimerId: ReturnType<typeof setInterval> | null;
  updateInFlight: boolean;
};

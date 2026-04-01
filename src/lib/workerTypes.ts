import type { HybridAStarPlanner, MpcReferenceTracker } from '../../wasm-core/pkg/wasm_core';

export type WasmCarState = {
    x: number;
    y: number;
    yaw: number;
    velocity: number;
    steer: number;
};

export type WorkerRequest = {
    id: number;
    type: string;
    payload?: unknown;
};

export type WorkerResponse = { id: number; ok: true; result: unknown } | { id: number; ok: false; error: string };

export type WorkerEvent = {
    type: string;
    payload?: unknown;
};

export type PlannerSession = {
    planner: HybridAStarPlanner;
    cancelled: boolean;
};

export type HybridSeedPoint = {
    x: number;
    y: number;
    yaw: number;
    velocity: number;
};

export type SimulationSession = {
    state: WasmCarState;
    timestamp: number;
    simDeltaTime: number;
    controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> | null;
    stopped: boolean;
    simulationTimerId: ReturnType<typeof setTimeout> | null;
    publishTimerId: ReturnType<typeof setInterval> | null;
    loopToken: number;
    stateVersion: number;
};

export type TrackingPlan = {
    path: Array<{ x: number; y: number; yaw: number }>;
    directions: number[];
};

export type LocalPlannerPathPoint = {
    x: number;
    y: number;
    yaw: number;
};

export type LocalPlannerReferencePoint = {
    x: number;
    y: number;
    velocity: number;
    yaw: number;
};

export type LocalPlannerUpdateResult = {
    controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>;
    localTrajectory: LocalPlannerPathPoint[];
    referencePoints: LocalPlannerReferencePoint[];
    brakeTrajectory: LocalPlannerReferencePoint[];
};

export type LocalPlannerSession = {
    tracker: MpcReferenceTracker | null;
    latestState: { state: WasmCarState; timestamp: number } | null;
    simDeltaTime: number;
    updateIntervalMs: number;
    updateTimerId: number | null;
    updateInFlight: boolean;
};

export const HYBRID_STEP_BUDGET = 96;
export const HYBRID_SEGMENT_BATCH_SIZE = 320;
export const DEFAULT_SIM_DELTA_TIME = 0.015;
export const DEFAULT_SIM_INTERVAL_MS = 20;
export const DEFAULT_PUBLISH_INTERVAL_MS = 50;
export const DEFAULT_LOCAL_PLANNER_DT = 0.07;
export const DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS = 100;

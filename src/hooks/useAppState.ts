import { useCallback, useEffect, useMemo, useReducer, useRef } from 'react';

import type { CarState } from '../lib/appModel';
import { FALLBACK_MAP_BOUNDING_BOX, type GoalUnreachableState, type MapServerSnapshot } from '../lib/appTypes';
import { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import { checkTrajectoryCollision } from '../lib/wasmCore';
import type { AppRefs, AppState, AppStateUpdater, HistoryPoint, StateUpdater } from './appRuntimeTypes';

function createInitialMapSnapshot(): MapServerSnapshot {
  return {
    boundingBox: FALLBACK_MAP_BOUNDING_BOX,
    knownObstacles: [],
    unknownObstacles: [],
  };
}

function createInitialGoalUnreachableState(): GoalUnreachableState {
  return {
    visible: false,
    x: 0,
    y: 0,
  };
}

function createInitialHistory(): HistoryPoint[] {
  return [{ t: 0, value: 0 }];
}

function createInitialState(): AppState {
  return {
    mode: 'goal',
    timestamp: 0,
    mapSnapshot: createInitialMapSnapshot(),
    carShape: null,
    motionLimits: null,
    car: null,
    goal: null,
    pressedPose: null,
    goalUnreachable: createInitialGoalUnreachableState(),
    globalTrajectory: null,
    localTrajectory: [],
    referencePoints: [],
    globalPlannerSegments: [],
    velocityHistory: createInitialHistory(),
    steerHistory: createInitialHistory(),
  };
}

type AppStateAction = {
  apply: (state: AppState) => AppState;
};

function reduceAppState(state: AppState, action: AppStateAction) {
  return action.apply(state);
}

function isUpdaterFunction<T>(updater: StateUpdater<T>): updater is (value: T) => T {
  return typeof updater === 'function';
}

function resolveUpdater<T>(current: T, updater: StateUpdater<T>): T {
  return isUpdaterFunction(updater) ? updater(current) : updater;
}

function buildStateAction<Key extends keyof AppState>(key: Key, updater: StateUpdater<AppState[Key]>): AppStateAction {
  return {
    apply: (state) => {
      const nextValue = resolveUpdater(state[key], updater);
      if (Object.is(nextValue, state[key])) {
        return state;
      }
      return {
        ...state,
        [key]: nextValue,
      };
    },
  };
}

export function useAppState() {
  const [state, dispatch] = useReducer(reduceAppState, undefined, createInitialState);

  const mapServerNodeRef = useRef(null);
  const carRef = useRef<CarState | null>(state.car);
  const timestampRef = useRef(state.timestamp);
  const goalRef = useRef<CarState | null>(state.goal);
  const mapSnapshotRef = useRef(state.mapSnapshot);
  const globalTrajectoryRef = useRef(state.globalTrajectory);
  const localPlanningRef = useRef(false);
  const brakeTrajectoryRef = useRef(null);
  const planningRequestRef = useRef(0);
  const dragStartRef = useRef(null);
  const trajectoryCollisionCheckingNodeRef = useRef<TrajectoryCollisionCheckingNode | null>(null);
  const dashboardGridRef = useRef<HTMLElement | null>(null);

  if (trajectoryCollisionCheckingNodeRef.current === null) {
    trajectoryCollisionCheckingNodeRef.current = new TrajectoryCollisionCheckingNode(checkTrajectoryCollision);
  }

  useEffect(() => {
    carRef.current = state.car;
    timestampRef.current = state.timestamp;
    goalRef.current = state.goal;
    mapSnapshotRef.current = state.mapSnapshot;
    globalTrajectoryRef.current = state.globalTrajectory;
  }, [state.car, state.timestamp, state.goal, state.mapSnapshot, state.globalTrajectory]);

  const refs = useMemo<AppRefs>(
    () => ({
      mapServerNodeRef,
      carRef,
      timestampRef,
      goalRef,
      mapSnapshotRef,
      globalTrajectoryRef,
      localPlanningRef,
      brakeTrajectoryRef,
      planningRequestRef,
      dragStartRef,
      trajectoryCollisionCheckingNodeRef,
    }),
    [],
  );

  const updateState = useCallback<AppStateUpdater>((key, updater) => {
    dispatch(buildStateAction(key, updater));
  }, []);

  return {
    state,
    refs,
    updateState,
    dashboardGridRef,
  };
}

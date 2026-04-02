import { useEffect, useMemo, useRef, useState } from 'react';

import type { CarState, Mode } from '../lib/appModel';
import {
  FALLBACK_MAP_BOUNDING_BOX,
  type CarShape,
  type DragStartState,
  type GoalUnreachableState,
  type MapServerSnapshot,
  type MotionLimits,
} from '../lib/appTypes';
import { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
import {
  checkTrajectoryCollision,
  type HybridAStarProgress,
  type LocalPlannerPathPoint,
  type LocalPlannerReferencePoint,
  type LocalPlannerTrajectoryPoint,
} from '../lib/wasmCore';
import type { AppRefs, AppSetters, HistoryPoint } from './appRuntimeTypes';

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

export function useAppState() {
  const [mode, setMode] = useState<Mode>('goal');
  const [timestamp, setTimestamp] = useState(0);
  const [mapSnapshot, setMapSnapshot] = useState<MapServerSnapshot>(createInitialMapSnapshot);
  const [carShape, setCarShape] = useState<CarShape | null>(null);
  const [motionLimits, setMotionLimits] = useState<MotionLimits | null>(null);
  const [car, setCar] = useState<CarState | null>(null);
  const [goal, setGoal] = useState<CarState | null>(null);
  const [pressedPose, setPressedPose] = useState<CarState | null>(null);
  const [goalUnreachable, setGoalUnreachable] = useState<GoalUnreachableState>(createInitialGoalUnreachableState);
  const [globalTrajectory, setGlobalTrajectory] = useState<LocalPlannerTrajectoryPoint[] | null>(null);
  const [localTrajectory, setLocalTrajectory] = useState<LocalPlannerPathPoint[]>([]);
  const [referencePoints, setReferencePoints] = useState<LocalPlannerReferencePoint[]>([]);
  const [globalPlannerSegments, setGlobalPlannerSegments] = useState<HybridAStarProgress['segments'][]>([]);
  const [velocityHistory, setVelocityHistory] = useState<HistoryPoint[]>(createInitialHistory);
  const [steerHistory, setSteerHistory] = useState<HistoryPoint[]>(createInitialHistory);

  const mapServerNodeRef = useRef(null);
  const carRef = useRef<CarState | null>(car);
  const timestampRef = useRef(timestamp);
  const goalRef = useRef<CarState | null>(goal);
  const mapSnapshotRef = useRef(mapSnapshot);
  const globalTrajectoryRef = useRef<LocalPlannerTrajectoryPoint[] | null>(globalTrajectory);
  const localPlanningRef = useRef(false);
  const brakeTrajectoryRef = useRef<LocalPlannerReferencePoint[] | null>(null);
  const planningRequestRef = useRef(0);
  const dragStartRef = useRef<DragStartState | null>(null);
  const trajectoryCollisionCheckingNodeRef = useRef<TrajectoryCollisionCheckingNode | null>(null);
  const dashboardGridRef = useRef<HTMLElement | null>(null);

  if (trajectoryCollisionCheckingNodeRef.current === null) {
    trajectoryCollisionCheckingNodeRef.current = new TrajectoryCollisionCheckingNode(checkTrajectoryCollision);
  }

  useEffect(() => {
    carRef.current = car;
    timestampRef.current = timestamp;
    goalRef.current = goal;
    mapSnapshotRef.current = mapSnapshot;
    globalTrajectoryRef.current = globalTrajectory;
  }, [car, timestamp, goal, mapSnapshot, globalTrajectory]);

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

  const setters = useMemo<AppSetters>(
    () => ({
      setMode,
      setTimestamp,
      setMapSnapshot,
      setCarShape,
      setMotionLimits,
      setCar,
      setGoal,
      setPressedPose,
      setGoalUnreachable,
      setGlobalTrajectory,
      setLocalTrajectory,
      setReferencePoints,
      setGlobalPlannerSegments,
      setVelocityHistory,
      setSteerHistory,
    }),
    [],
  );

  return {
    state: {
      mode,
      timestamp,
      mapSnapshot,
      carShape,
      motionLimits,
      car,
      goal,
      pressedPose,
      goalUnreachable,
      globalTrajectory,
      localTrajectory,
      referencePoints,
      globalPlannerSegments,
      velocityHistory,
      steerHistory,
    },
    refs,
    setters,
    dashboardGridRef,
  };
}

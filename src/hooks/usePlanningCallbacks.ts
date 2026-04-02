import { useCallback, useEffect } from 'react';

import type { CarState } from '../lib/appModel';
import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  resumeSimulationMotion,
  setLocalPlannerTrajectory,
  setSimulationState,
  solveHybridAStar,
  stopSimulationMotion,
} from '../lib/wasmCore';
import type { UsePlanningCallbacksParams } from './planningHelpers';

export function usePlanningCallbacks({
  mode,
  carRef,
  goalRef,
  mapSnapshotRef,
  globalTrajectoryRef,
  brakeTrajectoryRef,
  dragStartRef,
  planningRequestRef,
  localPlanningRef,
  trajectoryCollisionCheckingNodeRef,
  mapServerNodeRef,
  setCar,
  setGoal,
  setPressedPose,
  setGoalUnreachable,
  setGlobalTrajectory,
  setGlobalPlannerSegments,
  setLocalTrajectory,
  setReferencePoints,
  setMapSnapshot,
  replanMaxSpeed,
  toHybridAStarStartSeed,
}: UsePlanningCallbacksParams) {
  const clearGlobalPlannerDisplaySegments = useCallback(() => {
    setGlobalPlannerSegments([]);
  }, [setGlobalPlannerSegments]);

  const handleCancel = useCallback(async () => {
    planningRequestRef.current += 1;
    dragStartRef.current = null;
    setPressedPose(null);
    setGoalUnreachable((current) => ({ ...current, visible: false }));
    clearGlobalPlannerDisplaySegments();
    setLocalTrajectory([]);
    setReferencePoints([]);
    localPlanningRef.current = false;
    brakeTrajectoryRef.current = null;
    trajectoryCollisionCheckingNodeRef.current?.cancel();
    try {
      await Promise.all([stopSimulationMotion(), cancelHybridAStar(), cancelLocalPlanner()]);
    } catch (error) {
      console.error('Failed to cancel current execution', error);
    }
  }, [
    clearGlobalPlannerDisplaySegments,
    planningRequestRef,
    dragStartRef,
    setPressedPose,
    setGoalUnreachable,
    setLocalTrajectory,
    setReferencePoints,
    localPlanningRef,
    brakeTrajectoryRef,
    trajectoryCollisionCheckingNodeRef,
  ]);

  const handleBrake = useCallback(async () => {
    planningRequestRef.current += 1;
    setGoalUnreachable((current) => ({ ...current, visible: false }));
    clearGlobalPlannerDisplaySegments();
    trajectoryCollisionCheckingNodeRef.current?.cancel();
    try {
      await Promise.all([cancelHybridAStar(), brakeLocalPlanner()]);
    } catch (error) {
      console.error('Failed to brake current execution', error);
    }
  }, [clearGlobalPlannerDisplaySegments, planningRequestRef, setGoalUnreachable, trajectoryCollisionCheckingNodeRef]);

  const runGlobalPlan = useCallback(async () => {
    const measuredState = carRef.current;
    const goalState = goalRef.current;

    if (!measuredState || !goalState) {
      return;
    }

    const start =
      Math.abs(measuredState.velocity) > replanMaxSpeed && brakeTrajectoryRef.current
        ? toHybridAStarStartSeed(brakeTrajectoryRef.current)
        : measuredState;

    const requestId = planningRequestRef.current + 1;
    planningRequestRef.current = requestId;

    clearGlobalPlannerDisplaySegments();

    // Helper to check if this planning request has been cancelled
    const isCancelled = () => planningRequestRef.current !== requestId;

    try {
      const result = await solveHybridAStar(
        start,
        goalState,
        flattenObstacleCoordinates(mapSnapshotRef.current.knownObstacles),
        4000,
        requestId,
      );
      if (isCancelled()) return;

      clearGlobalPlannerDisplaySegments();
      if (!result) {
        setGlobalTrajectory(null);
        globalTrajectoryRef.current = null;
        localPlanningRef.current = false;
        const collisionChecker = trajectoryCollisionCheckingNodeRef.current;
        await Promise.all([
          collisionChecker ? collisionChecker.setTrajectory(null) : Promise.resolve(false),
          setLocalPlannerTrajectory(null),
        ]);
        setGoalUnreachable((current) => ({ ...current, visible: true }));
        return;
      }
      if (result.token !== requestId) {
        return;
      }
      const trajectory = result.path.map((point, index) => ({
        x: point.x,
        y: point.y,
        yaw: point.yaw,
        direction: result.directions[index] ?? 0,
      }));
      setGlobalTrajectory(trajectory);
      globalTrajectoryRef.current = trajectory;
      localPlanningRef.current = true;
      setGoalUnreachable((current) => ({ ...current, visible: false }));

      if (isCancelled()) return;
      await setLocalPlannerTrajectory(trajectory);
      if (isCancelled()) return;
      const collided = await trajectoryCollisionCheckingNodeRef.current!.setTrajectory(
        trajectory.map((point) => ({ x: point.x, y: point.y, yaw: point.yaw })),
      );
      if (isCancelled()) return;
      if (collided) {
        return;
      }

      await resumeSimulationMotion();
    } catch (error) {
      if (isCancelled()) {
        return;
      }
      if (error instanceof Error && error.message === 'Hybrid A* search cancelled') {
        return;
      }
      clearGlobalPlannerDisplaySegments();
      setGlobalTrajectory(null);
      globalTrajectoryRef.current = null;
      const collisionChecker = trajectoryCollisionCheckingNodeRef.current;
      await Promise.all([
        collisionChecker ? collisionChecker.setTrajectory(null) : Promise.resolve(false),
        setLocalPlannerTrajectory(null),
      ]);
      setGoalUnreachable((current) => ({ ...current, visible: true }));
      console.error('Failed to compute global plan', error);
    }
  }, [
    clearGlobalPlannerDisplaySegments,
    carRef,
    goalRef,
    brakeTrajectoryRef,
    planningRequestRef,
    replanMaxSpeed,
    toHybridAStarStartSeed,
    mapSnapshotRef,
    setGlobalTrajectory,
    globalTrajectoryRef,
    localPlanningRef,
    trajectoryCollisionCheckingNodeRef,
    setGoalUnreachable,
  ]);

  const handleTrajectoryCollided = useCallback(async () => {
    setGlobalTrajectory(null);
    globalTrajectoryRef.current = null;
    await runGlobalPlan();
  }, [runGlobalPlan, setGlobalTrajectory, globalTrajectoryRef]);

  useEffect(() => {
    const node = trajectoryCollisionCheckingNodeRef.current;
    if (!node) {
      return;
    }

    node.setCollidedListener(() => {
      void brakeLocalPlanner().catch((error) => {
        console.error('Failed to brake local planner after collision', error);
      });
      void handleTrajectoryCollided().catch((error) => {
        console.error('Failed to handle trajectory collision', error);
      });
    });

    return () => {
      node.setCollidedListener(null);
    };
  }, [handleTrajectoryCollided, trajectoryCollisionCheckingNodeRef]);

  const handleRestart = useCallback(async () => {
    await handleCancel();
    goalRef.current = null;
    globalTrajectoryRef.current = null;
    brakeTrajectoryRef.current = null;
    setGoal(null);
    setPressedPose(null);
    setGoalUnreachable((current) => ({ ...current, visible: false }));
    setGlobalTrajectory(null);

    try {
      const mapServerNode = mapServerNodeRef.current;
      if (!mapServerNode) {
        return;
      }

      const nextSnapshot = mapServerNode.init();
      mapSnapshotRef.current = nextSnapshot;
      trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(
        flattenObstacleCoordinates(nextSnapshot.knownObstacles),
      );
      setMapSnapshot(nextSnapshot);

      const nextCar = await mapServerNode.generateRandomInitialState();
      carRef.current = nextCar;
      setCar(nextCar);
      await setSimulationState(nextCar);
    } catch (error) {
      console.error('Failed to restart simulation state', error);
    }
  }, [
    handleCancel,
    goalRef,
    globalTrajectoryRef,
    brakeTrajectoryRef,
    setGoal,
    setPressedPose,
    setGoalUnreachable,
    setGlobalTrajectory,
    mapServerNodeRef,
    mapSnapshotRef,
    trajectoryCollisionCheckingNodeRef,
    setMapSnapshot,
    carRef,
    setCar,
  ]);

  const commitDrag = useCallback(
    async (finalX: number, finalY: number, startX: number, startY: number) => {
      const state: CarState = {
        x: startX,
        y: startY,
        yaw: Math.atan2(finalY - startY, finalX - startX),
        velocity: 0,
        steer: 0,
      };

      setPressedPose(null);
      setGlobalTrajectory(null);
      globalTrajectoryRef.current = null;

      if (mode === 'pose') {
        setGoal(null);
        goalRef.current = null;
        await handleCancel();
        try {
          await setSimulationState(state);
        } catch (error) {
          console.error('Failed to set simulation pose', error);
        }
        return;
      }

      setGoal(state);
      goalRef.current = state;
      setGoalUnreachable({ visible: false, x: startX, y: startY });
      await handleBrake();
      await runGlobalPlan();
    },
    [
      handleBrake,
      handleCancel,
      mode,
      runGlobalPlan,
      setPressedPose,
      setGlobalTrajectory,
      globalTrajectoryRef,
      setGoal,
      goalRef,
      setGoalUnreachable,
    ],
  );

  const handleMapPrimaryDragStart = useCallback(
    (world: { x: number; y: number }) => {
      const bounds = mapSnapshotRef.current.boundingBox;
      if (world.x < bounds.minX || world.x > bounds.maxX || world.y < bounds.minY || world.y > bounds.maxY) {
        return false;
      }

      setGoalUnreachable((current) => ({ ...current, visible: false }));
      dragStartRef.current = { startX: world.x, startY: world.y };
      setPressedPose({ x: world.x, y: world.y, yaw: 0, velocity: 0, steer: 0 });
      return true;
    },
    [mapSnapshotRef, setGoalUnreachable, dragStartRef, setPressedPose],
  );

  const handleMapPrimaryDragMove = useCallback(
    (world: { x: number; y: number }) => {
      const start = dragStartRef.current;
      if (!start) {
        return;
      }

      setPressedPose({
        x: start.startX,
        y: start.startY,
        yaw: Math.atan2(world.y - start.startY, world.x - start.startX),
        velocity: 0,
        steer: 0,
      });
    },
    [dragStartRef, setPressedPose],
  );

  const handleMapPrimaryDragEnd = useCallback(
    (world: { x: number; y: number }) => {
      const currentDrag = dragStartRef.current;
      if (!currentDrag) {
        return;
      }

      dragStartRef.current = null;
      setPressedPose(null);
      void commitDrag(world.x, world.y, currentDrag.startX, currentDrag.startY);
    },
    [commitDrag, dragStartRef, setPressedPose],
  );

  const handleMapPrimaryDragCancel = useCallback(() => {
    dragStartRef.current = null;
    setPressedPose(null);
  }, [dragStartRef, setPressedPose]);

  return {
    clearGlobalPlannerDisplaySegments,
    handleCancel,
    handleBrake,
    runGlobalPlan,
    handleTrajectoryCollided,
    handleRestart,
    commitDrag,
    handleMapPrimaryDragStart,
    handleMapPrimaryDragMove,
    handleMapPrimaryDragEnd,
    handleMapPrimaryDragCancel,
  };
}

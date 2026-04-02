import { useCallback, useEffect } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  resumeSimulationMotion,
  setLocalPlannerTrajectory,
  solveHybridAStar,
} from '../lib/wasmCore';
import { clearGoalPlanState, type UsePlanningCallbacksParams } from './planningHelpers';

type UseGlobalPlanningParams = Pick<
  UsePlanningCallbacksParams,
  'refs' | 'setters' | 'replanMaxSpeed' | 'toHybridAStarStartSeed'
>;

export function useGlobalPlanning({ refs, setters, replanMaxSpeed, toHybridAStarStartSeed }: UseGlobalPlanningParams) {
  const {
    brakeTrajectoryRef,
    carRef,
    globalTrajectoryRef,
    localPlanningRef,
    planningRequestRef,
    trajectoryCollisionCheckingNodeRef,
  } = refs;

  const clearGlobalPlannerDisplaySegments = useCallback(() => {
    setters.setGlobalPlannerSegments([]);
  }, [setters]);

  const runGlobalPlan = useCallback(async () => {
    const measuredState = carRef.current;
    const goalState = refs.goalRef.current;

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

    const isCancelled = () => planningRequestRef.current !== requestId;

    try {
      const result = await solveHybridAStar(
        start,
        goalState,
        flattenObstacleCoordinates(refs.mapSnapshotRef.current.knownObstacles),
        4000,
        requestId,
      );

      if (isCancelled()) {
        return;
      }

      clearGlobalPlannerDisplaySegments();
      if (!result) {
        globalTrajectoryRef.current = null;
        localPlanningRef.current = false;
        await Promise.all([
          trajectoryCollisionCheckingNodeRef.current?.setTrajectory(null) ?? Promise.resolve(false),
          setLocalPlannerTrajectory(null),
        ]);
        clearGoalPlanState(refs, setters, { visible: true });
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

      setters.setGlobalTrajectory(trajectory);
      globalTrajectoryRef.current = trajectory;
      localPlanningRef.current = true;
      setters.setGoalUnreachable((current) => ({ ...current, visible: false }));

      if (isCancelled()) {
        return;
      }

      await setLocalPlannerTrajectory(trajectory);
      if (isCancelled()) {
        return;
      }

      const collided = await trajectoryCollisionCheckingNodeRef.current?.setTrajectory(
        trajectory.map((point) => ({ x: point.x, y: point.y, yaw: point.yaw })),
      );

      if (isCancelled() || collided) {
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

      await Promise.all([
        trajectoryCollisionCheckingNodeRef.current?.setTrajectory(null) ?? Promise.resolve(false),
        setLocalPlannerTrajectory(null),
      ]);
      clearGlobalPlannerDisplaySegments();
      clearGoalPlanState(refs, setters, { visible: true });
      console.error('Failed to compute global plan', error);
    }
  }, [
    brakeTrajectoryRef,
    carRef,
    clearGlobalPlannerDisplaySegments,
    globalTrajectoryRef,
    localPlanningRef,
    planningRequestRef,
    refs,
    replanMaxSpeed,
    setters,
    toHybridAStarStartSeed,
    trajectoryCollisionCheckingNodeRef,
  ]);

  const handleTrajectoryCollided = useCallback(async () => {
    setters.setGlobalTrajectory(null);
    globalTrajectoryRef.current = null;
    await runGlobalPlan();
  }, [globalTrajectoryRef, runGlobalPlan, setters]);

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

  return {
    clearGlobalPlannerDisplaySegments,
    runGlobalPlan,
    handleTrajectoryCollided,
  };
}

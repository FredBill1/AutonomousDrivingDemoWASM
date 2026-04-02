import { useCallback, useEffect } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import { brakeLocalPlanner, resumeSimulationMotion, setLocalPlannerTrajectory, solveHybridAStar } from '../lib/wasmCore';
import { clearGoalPlanState, type UsePlanningCallbacksParams } from './planningHelpers';

type UseGlobalPlanningParams = Pick<UsePlanningCallbacksParams, 'refs' | 'setters' | 'replanMaxSpeed' | 'toHybridAStarStartSeed'>;

export function useGlobalPlanning({ refs, setters, replanMaxSpeed, toHybridAStarStartSeed }: UseGlobalPlanningParams) {
  const clearGlobalPlannerDisplaySegments = useCallback(() => {
    setters.setGlobalPlannerSegments([]);
  }, [setters]);

  const runGlobalPlan = useCallback(async () => {
    const measuredState = refs.carRef.current;
    const goalState = refs.goalRef.current;

    if (!measuredState || !goalState) {
      return;
    }

    const start =
      Math.abs(measuredState.velocity) > replanMaxSpeed && refs.brakeTrajectoryRef.current
        ? toHybridAStarStartSeed(refs.brakeTrajectoryRef.current)
        : measuredState;

    const requestId = refs.planningRequestRef.current + 1;
    refs.planningRequestRef.current = requestId;
    clearGlobalPlannerDisplaySegments();

    const isCancelled = () => refs.planningRequestRef.current !== requestId;

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
        refs.globalTrajectoryRef.current = null;
        refs.localPlanningRef.current = false;
        await Promise.all([
          refs.trajectoryCollisionCheckingNodeRef.current?.setTrajectory(null) ?? Promise.resolve(false),
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
      refs.globalTrajectoryRef.current = trajectory;
      refs.localPlanningRef.current = true;
      setters.setGoalUnreachable((current) => ({ ...current, visible: false }));

      if (isCancelled()) {
        return;
      }

      await setLocalPlannerTrajectory(trajectory);
      if (isCancelled()) {
        return;
      }

      const collided = await refs.trajectoryCollisionCheckingNodeRef.current?.setTrajectory(
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
        refs.trajectoryCollisionCheckingNodeRef.current?.setTrajectory(null) ?? Promise.resolve(false),
        setLocalPlannerTrajectory(null),
      ]);
      clearGlobalPlannerDisplaySegments();
      clearGoalPlanState(refs, setters, { visible: true });
      console.error('Failed to compute global plan', error);
    }
  }, [clearGlobalPlannerDisplaySegments, refs, replanMaxSpeed, setters, toHybridAStarStartSeed]);

  const handleTrajectoryCollided = useCallback(async () => {
    setters.setGlobalTrajectory(null);
    refs.globalTrajectoryRef.current = null;
    await runGlobalPlan();
  }, [refs, runGlobalPlan, setters]);

  useEffect(() => {
    const node = refs.trajectoryCollisionCheckingNodeRef.current;
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
  }, [handleTrajectoryCollided, refs]);

  return {
    clearGlobalPlannerDisplaySegments,
    runGlobalPlan,
    handleTrajectoryCollided,
  };
}

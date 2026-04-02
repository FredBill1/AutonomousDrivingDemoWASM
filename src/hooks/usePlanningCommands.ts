import { useCallback } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  setSimulationState,
  stopSimulationMotion,
} from '../lib/wasmCore';
import { hideGoalUnreachable, resetPlanningInteractionState, type PlanningControllerParams } from './planningHelpers';

type UsePlanningCommandsParams = Pick<PlanningControllerParams, 'refs' | 'updateState'> & {
  clearGlobalPlannerDisplaySegments: () => void;
};

export function usePlanningCommands({
  refs,
  updateState,
  clearGlobalPlannerDisplaySegments,
}: UsePlanningCommandsParams) {
  const {
    brakeTrajectoryRef,
    carRef,
    globalTrajectoryRef,
    goalRef,
    localPlanningRef,
    mapServerNodeRef,
    mapSnapshotRef,
    planningRequestRef,
    trajectoryCollisionCheckingNodeRef,
  } = refs;

  const handleCancel = useCallback(async () => {
    planningRequestRef.current += 1;
    resetPlanningInteractionState(refs, updateState);
    clearGlobalPlannerDisplaySegments();
    updateState('localTrajectory', []);
    updateState('referencePoints', []);
    localPlanningRef.current = false;
    brakeTrajectoryRef.current = null;
    trajectoryCollisionCheckingNodeRef.current?.cancel();

    try {
      await Promise.all([stopSimulationMotion(), cancelHybridAStar(), cancelLocalPlanner()]);
    } catch (error) {
      console.error('Failed to cancel current execution', error);
    }
  }, [
    brakeTrajectoryRef,
    clearGlobalPlannerDisplaySegments,
    localPlanningRef,
    planningRequestRef,
    refs,
    trajectoryCollisionCheckingNodeRef,
    updateState,
  ]);

  const handleBrake = useCallback(async () => {
    planningRequestRef.current += 1;
    hideGoalUnreachable(updateState);
    clearGlobalPlannerDisplaySegments();
    trajectoryCollisionCheckingNodeRef.current?.cancel();

    try {
      await Promise.all([cancelHybridAStar(), brakeLocalPlanner()]);
    } catch (error) {
      console.error('Failed to brake current execution', error);
    }
  }, [clearGlobalPlannerDisplaySegments, planningRequestRef, trajectoryCollisionCheckingNodeRef, updateState]);

  const handleRestart = useCallback(async () => {
    await handleCancel();
    goalRef.current = null;
    globalTrajectoryRef.current = null;
    brakeTrajectoryRef.current = null;
    updateState('goal', null);
    updateState('pressedPose', null);
    updateState('globalTrajectory', null);
    hideGoalUnreachable(updateState);

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
      updateState('mapSnapshot', nextSnapshot);

      const nextCar = await mapServerNode.generateRandomInitialState();
      carRef.current = nextCar;
      updateState('car', nextCar);
      await setSimulationState(nextCar);
    } catch (error) {
      console.error('Failed to restart simulation state', error);
    }
  }, [
    brakeTrajectoryRef,
    carRef,
    globalTrajectoryRef,
    goalRef,
    handleCancel,
    mapServerNodeRef,
    mapSnapshotRef,
    trajectoryCollisionCheckingNodeRef,
    updateState,
  ]);

  return {
    handleCancel,
    handleBrake,
    handleRestart,
  };
}

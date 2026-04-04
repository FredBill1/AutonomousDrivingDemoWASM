import { useCallback } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  setSimulationState,
  stopSimulationMotion,
} from '../lib/wasmCore';
import {
  hideGoalUnreachable,
  INITIAL_SIMULATION_TIMESTAMP,
  resetPlanningInteractionState,
  resetSimulationSessionState,
  type PlanningControllerParams,
} from './planningHelpers';

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
    resetSimulationSessionState(refs, updateState);

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
      await setSimulationState(nextCar, INITIAL_SIMULATION_TIMESTAMP);
    } catch (error) {
      console.error('Failed to restart simulation state', error);
    }
  }, [carRef, handleCancel, mapServerNodeRef, mapSnapshotRef, refs, trajectoryCollisionCheckingNodeRef, updateState]);

  return {
    handleCancel,
    handleBrake,
    handleRestart,
  };
}

import { useCallback } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  setSimulationState,
  stopSimulationMotion,
} from '../lib/wasmCore';
import { resetPlanningInteractionState, type UsePlanningCallbacksParams } from './planningHelpers';

type UsePlanningCommandsParams = Pick<UsePlanningCallbacksParams, 'refs' | 'setters'> & {
  clearGlobalPlannerDisplaySegments: () => void;
};

export function usePlanningCommands({ refs, setters, clearGlobalPlannerDisplaySegments }: UsePlanningCommandsParams) {
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
    resetPlanningInteractionState(refs, setters);
    clearGlobalPlannerDisplaySegments();
    setters.setLocalTrajectory([]);
    setters.setReferencePoints([]);
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
    setters,
    trajectoryCollisionCheckingNodeRef,
  ]);

  const handleBrake = useCallback(async () => {
    planningRequestRef.current += 1;
    setters.setGoalUnreachable((current) => ({ ...current, visible: false }));
    clearGlobalPlannerDisplaySegments();
    trajectoryCollisionCheckingNodeRef.current?.cancel();

    try {
      await Promise.all([cancelHybridAStar(), brakeLocalPlanner()]);
    } catch (error) {
      console.error('Failed to brake current execution', error);
    }
  }, [clearGlobalPlannerDisplaySegments, planningRequestRef, setters, trajectoryCollisionCheckingNodeRef]);

  const handleRestart = useCallback(async () => {
    await handleCancel();
    goalRef.current = null;
    globalTrajectoryRef.current = null;
    brakeTrajectoryRef.current = null;
    setters.setGoal(null);
    setters.setPressedPose(null);
    setters.setGlobalTrajectory(null);
    setters.setGoalUnreachable((current) => ({ ...current, visible: false }));

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
      setters.setMapSnapshot(nextSnapshot);

      const nextCar = await mapServerNode.generateRandomInitialState();
      carRef.current = nextCar;
      setters.setCar(nextCar);
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
    setters,
    trajectoryCollisionCheckingNodeRef,
  ]);

  return {
    handleCancel,
    handleBrake,
    handleRestart,
  };
}

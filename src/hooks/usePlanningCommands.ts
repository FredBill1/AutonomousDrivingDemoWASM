import { useCallback } from 'react';

import { flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  brakeLocalPlanner,
  cancelHybridAStar,
  cancelLocalPlanner,
  setSimulationState,
  stopSimulationMotion,
} from '../lib/wasmCore';
import { clearGoalPlanState, resetPlanningInteractionState, type UsePlanningCallbacksParams } from './planningHelpers';

type UsePlanningCommandsParams = Pick<UsePlanningCallbacksParams, 'refs' | 'setters'> & {
  clearGlobalPlannerDisplaySegments: () => void;
};

export function usePlanningCommands({
  refs,
  setters,
  clearGlobalPlannerDisplaySegments,
}: UsePlanningCommandsParams) {
  const handleCancel = useCallback(async () => {
    refs.planningRequestRef.current += 1;
    resetPlanningInteractionState(refs, setters);
    clearGlobalPlannerDisplaySegments();
    setters.setLocalTrajectory([]);
    setters.setReferencePoints([]);
    refs.localPlanningRef.current = false;
    refs.brakeTrajectoryRef.current = null;
    refs.trajectoryCollisionCheckingNodeRef.current?.cancel();

    try {
      await Promise.all([stopSimulationMotion(), cancelHybridAStar(), cancelLocalPlanner()]);
    } catch (error) {
      console.error('Failed to cancel current execution', error);
    }
  }, [clearGlobalPlannerDisplaySegments, refs, setters]);

  const handleBrake = useCallback(async () => {
    refs.planningRequestRef.current += 1;
    setters.setGoalUnreachable((current) => ({ ...current, visible: false }));
    clearGlobalPlannerDisplaySegments();
    refs.trajectoryCollisionCheckingNodeRef.current?.cancel();

    try {
      await Promise.all([cancelHybridAStar(), brakeLocalPlanner()]);
    } catch (error) {
      console.error('Failed to brake current execution', error);
    }
  }, [clearGlobalPlannerDisplaySegments, refs, setters]);

  const handleRestart = useCallback(async () => {
    await handleCancel();
    refs.goalRef.current = null;
    refs.globalTrajectoryRef.current = null;
    refs.brakeTrajectoryRef.current = null;
    setters.setGoal(null);
    setters.setPressedPose(null);
    setters.setGlobalTrajectory(null);
    setters.setGoalUnreachable((current) => ({ ...current, visible: false }));

    try {
      const mapServerNode = refs.mapServerNodeRef.current;
      if (!mapServerNode) {
        return;
      }

      const nextSnapshot = mapServerNode.init();
      refs.mapSnapshotRef.current = nextSnapshot;
      refs.trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(
        flattenObstacleCoordinates(nextSnapshot.knownObstacles),
      );
      setters.setMapSnapshot(nextSnapshot);

      const nextCar = await mapServerNode.generateRandomInitialState();
      refs.carRef.current = nextCar;
      setters.setCar(nextCar);
      await setSimulationState(nextCar);
    } catch (error) {
      console.error('Failed to restart simulation state', error);
    }
  }, [handleCancel, refs, setters]);

  return {
    handleCancel,
    handleBrake,
    handleRestart,
  };
}

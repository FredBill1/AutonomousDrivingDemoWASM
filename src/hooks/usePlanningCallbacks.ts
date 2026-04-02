import type { UsePlanningCallbacksParams } from './planningHelpers';
import { useGlobalPlanning } from './useGlobalPlanning';
import { usePlanningCommands } from './usePlanningCommands';
import { usePlanningDrag } from './usePlanningDrag';

export function usePlanningCallbacks({
  mode,
  refs,
  setters,
  replanMaxSpeed,
  toHybridAStarStartSeed,
}: UsePlanningCallbacksParams) {
  const { clearGlobalPlannerDisplaySegments, runGlobalPlan, handleTrajectoryCollided } = useGlobalPlanning({
    refs,
    setters,
    replanMaxSpeed,
    toHybridAStarStartSeed,
  });
  const { handleCancel, handleBrake, handleRestart } = usePlanningCommands({
    refs,
    setters,
    clearGlobalPlannerDisplaySegments,
  });
  const {
    commitDrag,
    handleMapPrimaryDragStart,
    handleMapPrimaryDragMove,
    handleMapPrimaryDragEnd,
    handleMapPrimaryDragCancel,
  } = usePlanningDrag({
    mode,
    refs,
    setters,
    handleBrake,
    handleCancel,
    runGlobalPlan,
  });

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

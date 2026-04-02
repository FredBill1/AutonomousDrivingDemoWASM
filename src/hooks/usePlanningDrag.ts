import { useCallback } from 'react';

import type { CarState } from '../lib/appModel';
import { setSimulationState } from '../lib/wasmCore';
import { hideGoalUnreachable, type PlanningControllerParams } from './planningHelpers';

type UsePlanningDragParams = Pick<PlanningControllerParams, 'mode' | 'refs' | 'updateState'> & {
  handleBrake: () => Promise<void>;
  handleCancel: () => Promise<void>;
  runGlobalPlan: () => Promise<void>;
};

export function usePlanningDrag({
  mode,
  refs,
  updateState,
  handleBrake,
  handleCancel,
  runGlobalPlan,
}: UsePlanningDragParams) {
  const { dragStartRef, globalTrajectoryRef, goalRef, mapSnapshotRef } = refs;

  const commitDrag = useCallback(
    async (finalX: number, finalY: number, startX: number, startY: number) => {
      const state: CarState = {
        x: startX,
        y: startY,
        yaw: Math.atan2(finalY - startY, finalX - startX),
        velocity: 0,
        steer: 0,
      };

      updateState('pressedPose', null);
      updateState('globalTrajectory', null);
      globalTrajectoryRef.current = null;

      if (mode === 'pose') {
        updateState('goal', null);
        goalRef.current = null;
        await handleCancel();
        try {
          await setSimulationState(state);
        } catch (error) {
          console.error('Failed to set simulation pose', error);
        }
        return;
      }

      updateState('goal', state);
      goalRef.current = state;
      updateState('goalUnreachable', { visible: false, x: startX, y: startY });
      await handleBrake();
      await runGlobalPlan();
    },
    [globalTrajectoryRef, goalRef, handleBrake, handleCancel, mode, runGlobalPlan, updateState],
  );

  const handleMapPrimaryDragStart = useCallback(
    (world: { x: number; y: number }) => {
      const bounds = mapSnapshotRef.current.boundingBox;
      if (world.x < bounds.minX || world.x > bounds.maxX || world.y < bounds.minY || world.y > bounds.maxY) {
        return false;
      }

      hideGoalUnreachable(updateState);
      dragStartRef.current = { startX: world.x, startY: world.y };
      updateState('pressedPose', { x: world.x, y: world.y, yaw: 0, velocity: 0, steer: 0 });
      return true;
    },
    [dragStartRef, mapSnapshotRef, updateState],
  );

  const handleMapPrimaryDragMove = useCallback(
    (world: { x: number; y: number }) => {
      const start = dragStartRef.current;
      if (!start) {
        return;
      }

      updateState('pressedPose', {
        x: start.startX,
        y: start.startY,
        yaw: Math.atan2(world.y - start.startY, world.x - start.startX),
        velocity: 0,
        steer: 0,
      });
    },
    [dragStartRef, updateState],
  );

  const handleMapPrimaryDragEnd = useCallback(
    (world: { x: number; y: number }) => {
      const currentDrag = dragStartRef.current;
      if (!currentDrag) {
        return;
      }

      dragStartRef.current = null;
      updateState('pressedPose', null);
      void commitDrag(world.x, world.y, currentDrag.startX, currentDrag.startY);
    },
    [commitDrag, dragStartRef, updateState],
  );

  const handleMapPrimaryDragCancel = useCallback(() => {
    dragStartRef.current = null;
    updateState('pressedPose', null);
  }, [dragStartRef, updateState]);

  return {
    commitDrag,
    handleMapPrimaryDragStart,
    handleMapPrimaryDragMove,
    handleMapPrimaryDragEnd,
    handleMapPrimaryDragCancel,
  };
}

import { useCallback } from 'react';

import type { CarState } from '../lib/appModel';
import { setSimulationState } from '../lib/wasmCore';
import type { UsePlanningCallbacksParams } from './planningHelpers';

type UsePlanningDragParams = Pick<UsePlanningCallbacksParams, 'mode' | 'refs' | 'setters'> & {
  handleBrake: () => Promise<void>;
  handleCancel: () => Promise<void>;
  runGlobalPlan: () => Promise<void>;
};

export function usePlanningDrag({
  mode,
  refs,
  setters,
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

      setters.setPressedPose(null);
      setters.setGlobalTrajectory(null);
      globalTrajectoryRef.current = null;

      if (mode === 'pose') {
        setters.setGoal(null);
        goalRef.current = null;
        await handleCancel();
        try {
          await setSimulationState(state);
        } catch (error) {
          console.error('Failed to set simulation pose', error);
        }
        return;
      }

      setters.setGoal(state);
      goalRef.current = state;
      setters.setGoalUnreachable({ visible: false, x: startX, y: startY });
      await handleBrake();
      await runGlobalPlan();
    },
    [globalTrajectoryRef, goalRef, handleBrake, handleCancel, mode, runGlobalPlan, setters],
  );

  const handleMapPrimaryDragStart = useCallback(
    (world: { x: number; y: number }) => {
      const bounds = mapSnapshotRef.current.boundingBox;
      if (world.x < bounds.minX || world.x > bounds.maxX || world.y < bounds.minY || world.y > bounds.maxY) {
        return false;
      }

      setters.setGoalUnreachable((current) => ({ ...current, visible: false }));
      dragStartRef.current = { startX: world.x, startY: world.y };
      setters.setPressedPose({ x: world.x, y: world.y, yaw: 0, velocity: 0, steer: 0 });
      return true;
    },
    [dragStartRef, mapSnapshotRef, setters],
  );

  const handleMapPrimaryDragMove = useCallback(
    (world: { x: number; y: number }) => {
      const start = dragStartRef.current;
      if (!start) {
        return;
      }

      setters.setPressedPose({
        x: start.startX,
        y: start.startY,
        yaw: Math.atan2(world.y - start.startY, world.x - start.startX),
        velocity: 0,
        steer: 0,
      });
    },
    [dragStartRef, setters],
  );

  const handleMapPrimaryDragEnd = useCallback(
    (world: { x: number; y: number }) => {
      const currentDrag = dragStartRef.current;
      if (!currentDrag) {
        return;
      }

      dragStartRef.current = null;
      setters.setPressedPose(null);
      void commitDrag(world.x, world.y, currentDrag.startX, currentDrag.startY);
    },
    [commitDrag, dragStartRef, setters],
  );

  const handleMapPrimaryDragCancel = useCallback(() => {
    dragStartRef.current = null;
    setters.setPressedPose(null);
  }, [dragStartRef, setters]);

  return {
    commitDrag,
    handleMapPrimaryDragStart,
    handleMapPrimaryDragMove,
    handleMapPrimaryDragEnd,
    handleMapPrimaryDragCancel,
  };
}

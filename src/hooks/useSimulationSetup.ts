import { useEffect } from 'react';

import { createCarShape, createMotionLimits } from '../lib/appHelpers';
import { MapServerNode, flattenObstacleCoordinates } from '../lib/mapServerNode';
import {
  checkCollision,
  ensureWasmCore,
  getCarConfigSnapshot,
  initSimulation,
  resetComputeWorker,
  setHybridAStarProgressListener,
  setLocalPlannerState,
  setLocalPlannerUpdateListener,
  setSimulationStateListener,
  stopSimulation,
} from '../lib/wasmCore';
import type { AppRefs, AppSetters } from './appRuntimeTypes';

type UseSimulationSetupParams = {
  refs: AppRefs;
  setters: AppSetters;
  historyLimit: number;
  localPlannerUpdateIntervalMs: number;
  maxGlobalPlannerDisplayBatches: number;
};

export function useSimulationSetup({
  refs,
  setters,
  historyLimit,
  localPlannerUpdateIntervalMs,
  maxGlobalPlannerDisplayBatches,
}: UseSimulationSetupParams): void {
  const {
    brakeTrajectoryRef,
    carRef,
    localPlanningRef,
    mapServerNodeRef,
    mapSnapshotRef,
    planningRequestRef,
    timestampRef,
    trajectoryCollisionCheckingNodeRef,
  } = refs;

  useEffect(() => {
    setHybridAStarProgressListener((progress) => {
      if (progress.token !== planningRequestRef.current) {
        return;
      }
      setters.setGlobalPlannerSegments((segments) => {
        const nextSegments = [...segments, progress.segments];
        return nextSegments.length > maxGlobalPlannerDisplayBatches
          ? nextSegments.slice(-maxGlobalPlannerDisplayBatches)
          : nextSegments;
      });
    });
    return () => setHybridAStarProgressListener(null);
  }, [maxGlobalPlannerDisplayBatches, planningRequestRef, setters]);

  useEffect(() => {
    let active = true;

    setLocalPlannerUpdateListener((result) => {
      if (!active || !localPlanningRef.current) {
        return;
      }

      brakeTrajectoryRef.current = result.brakeTrajectory;
      setters.setLocalTrajectory(result.localTrajectory);
      setters.setReferencePoints(result.referencePoints);
    });

    setSimulationStateListener((event) => {
      if (!active) {
        return;
      }

      void setLocalPlannerState(event.state, event.timestamp, localPlannerUpdateIntervalMs).catch((error) => {
        console.error('Failed to update local planner state', error);
      });

      carRef.current = event.state;
      timestampRef.current = event.timestamp;
      setters.setTimestamp(event.timestamp);
      setters.setCar(event.state);

      setters.setVelocityHistory((history) => [
        ...history.slice(-historyLimit + 1),
        { t: event.timestamp, value: event.state.velocity * 3.6 },
      ]);
      setters.setSteerHistory((history) => [
        ...history.slice(-historyLimit + 1),
        { t: event.timestamp, value: (event.state.steer * 180) / Math.PI },
      ]);

      const mapUpdate = mapServerNodeRef.current?.update(event.state);
      if (mapUpdate && mapUpdate.newObstacles.length > 0) {
        mapSnapshotRef.current = mapUpdate;
        trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(
          flattenObstacleCoordinates(mapUpdate.knownObstacles),
        );
        setters.setMapSnapshot(mapUpdate);
        void trajectoryCollisionCheckingNodeRef.current
          ?.checkCollision(flattenObstacleCoordinates(mapUpdate.newObstacles))
          .catch((error) => {
            console.error('Failed to check trajectory collision', error);
          });
      }
    });

    void (async () => {
      try {
        await ensureWasmCore();
        if (!active) {
          return;
        }

        const configSnapshot = await getCarConfigSnapshot();
        if (!active) {
          return;
        }

        setters.setCarShape(createCarShape(configSnapshot));
        setters.setMotionLimits(createMotionLimits(configSnapshot));

        let mapServerNode = mapServerNodeRef.current;
        if (mapServerNode === null) {
          mapServerNode = new MapServerNode(checkCollision, {
            backToCenter: configSnapshot.backToCenter,
            scanRadius: configSnapshot.scanRadius,
          });
          mapServerNodeRef.current = mapServerNode;
        } else {
          mapServerNode.setConfig({
            backToCenter: configSnapshot.backToCenter,
            scanRadius: configSnapshot.scanRadius,
          });
        }

        const snapshot = mapServerNode.init();
        if (!active) {
          return;
        }
        trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(
          flattenObstacleCoordinates(snapshot.knownObstacles),
        );
        setters.setMapSnapshot(snapshot);

        const initialCar = await mapServerNode.generateRandomInitialState();
        if (!active) {
          return;
        }

        carRef.current = initialCar;
        timestampRef.current = 0;
        setters.setCar(initialCar);
        setters.setTimestamp(0);

        await initSimulation(initialCar, 0);
      } catch (error) {
        console.error('Failed to initialize app state', error);
      }
    })();

    return () => {
      active = false;
      setLocalPlannerUpdateListener(null);
      setSimulationStateListener(null);
      void stopSimulation().catch((error) => {
        console.error('Failed to stop simulation during cleanup', error);
      });
      resetComputeWorker('App unmounted');
    };
  }, [
    brakeTrajectoryRef,
    carRef,
    setters,
    localPlannerUpdateIntervalMs,
    historyLimit,
    mapServerNodeRef,
    mapSnapshotRef,
    localPlanningRef,
    planningRequestRef,
    timestampRef,
    trajectoryCollisionCheckingNodeRef,
  ]);
}

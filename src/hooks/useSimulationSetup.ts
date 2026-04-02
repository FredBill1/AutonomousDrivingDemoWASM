import type React from 'react';
import { useEffect } from 'react';

import { createCarShape, createMotionLimits } from '../lib/appHelpers';
import type { CarState } from '../lib/appModel';
import type { CarShape, MapServerSnapshot, MotionLimits } from '../lib/appTypes';
import { MapServerNode, flattenObstacleCoordinates } from '../lib/mapServerNode';
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';
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
  type HybridAStarProgress,
  type LocalPlannerPathPoint,
  type LocalPlannerReferencePoint,
} from '../lib/wasmCore';

type UseSimulationSetupParams = {
  planningRequestRef: React.RefObject<number>;
  mapServerNodeRef: React.RefObject<MapServerNode | null>;
  trajectoryCollisionCheckingNodeRef: React.RefObject<TrajectoryCollisionCheckingNode | null>;
  carRef: React.RefObject<CarState | null>;
  timestampRef: React.RefObject<number>;
  localPlanningRef: React.RefObject<boolean>;
  brakeTrajectoryRef: React.RefObject<LocalPlannerReferencePoint[] | null>;
  mapSnapshotRef: React.RefObject<MapServerSnapshot>;
  setGlobalPlannerSegments: React.Dispatch<React.SetStateAction<HybridAStarProgress['segments'][]>>;
  setLocalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerPathPoint[]>>;
  setReferencePoints: React.Dispatch<React.SetStateAction<LocalPlannerReferencePoint[]>>;
  setMapSnapshot: React.Dispatch<React.SetStateAction<MapServerSnapshot>>;
  setCar: React.Dispatch<React.SetStateAction<CarState | null>>;
  setTimestamp: React.Dispatch<React.SetStateAction<number>>;
  setVelocityHistory: React.Dispatch<React.SetStateAction<{ t: number; value: number }[]>>;
  setSteerHistory: React.Dispatch<React.SetStateAction<{ t: number; value: number }[]>>;
  setCarShape: React.Dispatch<React.SetStateAction<CarShape | null>>;
  setMotionLimits: React.Dispatch<React.SetStateAction<MotionLimits | null>>;
  historyLimit: number;
  localPlannerUpdateIntervalMs: number;
  maxGlobalPlannerDisplayBatches: number;
};

export function useSimulationSetup({
  planningRequestRef,
  mapServerNodeRef,
  trajectoryCollisionCheckingNodeRef,
  carRef,
  timestampRef,
  localPlanningRef,
  brakeTrajectoryRef,
  mapSnapshotRef,
  setGlobalPlannerSegments,
  setLocalTrajectory,
  setReferencePoints,
  setMapSnapshot,
  setCar,
  setTimestamp,
  setVelocityHistory,
  setSteerHistory,
  setCarShape,
  setMotionLimits,
  historyLimit,
  localPlannerUpdateIntervalMs,
  maxGlobalPlannerDisplayBatches,
}: UseSimulationSetupParams): void {
  useEffect(() => {
    setHybridAStarProgressListener((progress) => {
      if (progress.token !== planningRequestRef.current) {
        return;
      }
      setGlobalPlannerSegments((segments) => {
        const nextSegments = [...segments, progress.segments];
        return nextSegments.length > maxGlobalPlannerDisplayBatches
          ? nextSegments.slice(-maxGlobalPlannerDisplayBatches)
          : nextSegments;
      });
    });
    return () => setHybridAStarProgressListener(null);
  }, [planningRequestRef, setGlobalPlannerSegments, maxGlobalPlannerDisplayBatches]);

  useEffect(() => {
    let active = true;

    setLocalPlannerUpdateListener((result) => {
      if (!active || !localPlanningRef.current) {
        return;
      }

      brakeTrajectoryRef.current = result.brakeTrajectory;
      setLocalTrajectory(result.localTrajectory);
      setReferencePoints(result.referencePoints);
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
      setTimestamp(event.timestamp);
      setCar(event.state);

      setVelocityHistory((history) => [
        ...history.slice(-historyLimit + 1),
        { t: event.timestamp, value: event.state.velocity * 3.6 },
      ]);
      setSteerHistory((history) => [
        ...history.slice(-historyLimit + 1),
        { t: event.timestamp, value: (event.state.steer * 180) / Math.PI },
      ]);

      const mapUpdate = mapServerNodeRef.current?.update(event.state);
      if (mapUpdate && mapUpdate.newObstacles.length > 0) {
        mapSnapshotRef.current = mapUpdate;
        trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(
          flattenObstacleCoordinates(mapUpdate.knownObstacles),
        );
        setMapSnapshot(mapUpdate);
        void trajectoryCollisionCheckingNodeRef.current
          ?.checkCollision(flattenObstacleCoordinates(mapUpdate.newObstacles))
          .then((collided) => {
            if (!collided) {
              return;
            }
          })
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

        setCarShape(createCarShape(configSnapshot));
        setMotionLimits(createMotionLimits(configSnapshot));

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
        setMapSnapshot(snapshot);

        const initialCar = await mapServerNode.generateRandomInitialState();
        if (!active) {
          return;
        }

        carRef.current = initialCar;
        timestampRef.current = 0;
        setCar(initialCar);
        setTimestamp(0);

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
    localPlanningRef,
    brakeTrajectoryRef,
    setLocalTrajectory,
    setReferencePoints,
    localPlannerUpdateIntervalMs,
    carRef,
    timestampRef,
    setTimestamp,
    setCar,
    setCarShape,
    setMotionLimits,
    setVelocityHistory,
    setSteerHistory,
    historyLimit,
    mapServerNodeRef,
    mapSnapshotRef,
    trajectoryCollisionCheckingNodeRef,
    setMapSnapshot,
  ]);
}

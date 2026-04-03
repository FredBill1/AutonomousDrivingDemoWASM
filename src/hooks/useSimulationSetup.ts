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
  setLocalPlannerUpdateListener,
  setSimulationStateListener,
  stopSimulation,
  type HybridAStarProgress,
  type LocalPlannerUpdateResult,
  type SimulationStateEvent,
  type WasmConfigSnapshot,
} from '../lib/wasmCore';
import type { AppRefs, AppStateUpdater, HistoryPoint } from './appRuntimeTypes';

const INITIAL_TIMESTAMP = 0;

type UseSimulationSetupParams = {
  refs: AppRefs;
  updateState: AppStateUpdater;
  historyLimit: number;
  maxGlobalPlannerDisplayBatches: number;
};

function appendHistory(history: HistoryPoint[], nextPoint: HistoryPoint, historyLimit: number) {
  return [...history.slice(-historyLimit + 1), nextPoint];
}

function syncKnownObstacles(refs: AppRefs, knownObstacles: AppRefs['mapSnapshotRef']['current']['knownObstacles']) {
  refs.trajectoryCollisionCheckingNodeRef.current?.setKnownObstacles(flattenObstacleCoordinates(knownObstacles));
}

function handleProgressUpdate(
  progress: HybridAStarProgress,
  refs: AppRefs,
  updateState: AppStateUpdater,
  maxGlobalPlannerDisplayBatches: number,
) {
  if (progress.token !== refs.planningRequestRef.current) {
    return;
  }

  updateState('globalPlannerSegments', (segments) => {
    const nextSegments = [...segments, progress.segments];
    return nextSegments.length > maxGlobalPlannerDisplayBatches
      ? nextSegments.slice(-maxGlobalPlannerDisplayBatches)
      : nextSegments;
  });
}

function handleLocalPlannerUpdate(result: LocalPlannerUpdateResult, refs: AppRefs, updateState: AppStateUpdater) {
  if (!refs.localPlanningRef.current) {
    return;
  }

  refs.brakeTrajectoryRef.current = result.brakeTrajectory;
  updateState('localTrajectory', result.localTrajectory);
  updateState('referencePoints', result.referencePoints);
}

function handleSimulationState(
  event: SimulationStateEvent,
  refs: AppRefs,
  updateState: AppStateUpdater,
  historyLimit: number,
) {
  refs.carRef.current = event.state;
  refs.timestampRef.current = event.timestamp;
  updateState('timestamp', event.timestamp);
  updateState('car', event.state);
  updateState('velocityHistory', (history) =>
    appendHistory(history, { t: event.timestamp, value: event.state.velocity * 3.6 }, historyLimit),
  );
  updateState('steerHistory', (history) =>
    appendHistory(history, { t: event.timestamp, value: (event.state.steer * 180) / Math.PI }, historyLimit),
  );

  const mapUpdate = refs.mapServerNodeRef.current?.update(event.state);
  if (!mapUpdate || mapUpdate.newObstacles.length === 0) {
    return;
  }

  refs.mapSnapshotRef.current = mapUpdate;
  syncKnownObstacles(refs, mapUpdate.knownObstacles);
  updateState('mapSnapshot', mapUpdate);
  void refs.trajectoryCollisionCheckingNodeRef.current
    ?.checkCollision(flattenObstacleCoordinates(mapUpdate.newObstacles))
    .catch((error) => {
      console.error('Failed to check trajectory collision', error);
    });
}

function createMapServerNode(snapshot: WasmConfigSnapshot) {
  return new MapServerNode(checkCollision, {
    backToCenter: snapshot.backToCenter,
    scanRadius: snapshot.scanRadius,
  });
}

async function initializeSimulationState(refs: AppRefs, updateState: AppStateUpdater, isActive: () => boolean) {
  await ensureWasmCore();
  if (!isActive()) {
    return;
  }
  const configSnapshot = await getCarConfigSnapshot();
  if (!isActive()) {
    return;
  }

  updateState('carShape', createCarShape(configSnapshot));
  updateState('motionLimits', createMotionLimits(configSnapshot));

  let mapServerNode = refs.mapServerNodeRef.current;
  if (mapServerNode === null) {
    mapServerNode = createMapServerNode(configSnapshot);
    refs.mapServerNodeRef.current = mapServerNode;
  } else {
    mapServerNode.setConfig({
      backToCenter: configSnapshot.backToCenter,
      scanRadius: configSnapshot.scanRadius,
    });
  }

  const snapshot = mapServerNode.init();
  if (!isActive()) {
    return;
  }
  refs.mapSnapshotRef.current = snapshot;
  syncKnownObstacles(refs, snapshot.knownObstacles);
  updateState('mapSnapshot', snapshot);

  const initialCar = await mapServerNode.generateRandomInitialState();
  if (!isActive()) {
    return;
  }
  refs.carRef.current = initialCar;
  refs.timestampRef.current = INITIAL_TIMESTAMP;
  updateState('car', initialCar);
  updateState('timestamp', INITIAL_TIMESTAMP);
  await initSimulation(initialCar, INITIAL_TIMESTAMP);
}

export function useSimulationSetup({
  refs,
  updateState,
  historyLimit,
  maxGlobalPlannerDisplayBatches,
}: UseSimulationSetupParams): void {
  useEffect(() => {
    setHybridAStarProgressListener((progress) => {
      handleProgressUpdate(progress, refs, updateState, maxGlobalPlannerDisplayBatches);
    });
    return () => setHybridAStarProgressListener(null);
  }, [maxGlobalPlannerDisplayBatches, refs, updateState]);

  useEffect(() => {
    let active = true;

    setLocalPlannerUpdateListener((result) => {
      if (!active) {
        return;
      }
      handleLocalPlannerUpdate(result, refs, updateState);
    });

    setSimulationStateListener((event) => {
      if (!active) {
        return;
      }
      handleSimulationState(event, refs, updateState, historyLimit);
    });

    void initializeSimulationState(refs, updateState, () => active).catch((error) => {
      if (active) {
        console.error('Failed to initialize app state', error);
      }
    });

    return () => {
      active = false;
      setLocalPlannerUpdateListener(null);
      setSimulationStateListener(null);
      void stopSimulation().catch((error) => {
        console.error('Failed to stop simulation during cleanup', error);
      });
      resetComputeWorker('App unmounted');
    };
  }, [historyLimit, refs, updateState]);
}

import { useEffect } from 'react';
import type React from 'react';

import type { CarState } from '../lib/appModel';
import { flattenObstacleCoordinates, type MapServerNode } from '../lib/mapServerNode';
import type { MapServerSnapshot } from '../lib/appTypes';
import {
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
import type { TrajectoryCollisionCheckingNode } from '../lib/trajectoryCollisionCheckingNode';

type UseSimulationSetupParams = {
    planningRequestRef: React.MutableRefObject<number>;
    mapServerNodeRef: React.RefObject<MapServerNode | null>;
    trajectoryCollisionCheckingNodeRef: React.MutableRefObject<TrajectoryCollisionCheckingNode | null>;
    carRef: React.MutableRefObject<CarState | null>;
    timestampRef: React.MutableRefObject<number>;
    localPlanningRef: React.MutableRefObject<boolean>;
    brakeTrajectoryRef: React.MutableRefObject<LocalPlannerReferencePoint[] | null>;
    setGlobalPlannerSegments: React.Dispatch<React.SetStateAction<HybridAStarProgress['segments'][]>>;
    setLocalTrajectory: React.Dispatch<React.SetStateAction<LocalPlannerPathPoint[]>>;
    setReferencePoints: React.Dispatch<React.SetStateAction<LocalPlannerReferencePoint[]>>;
    setMapSnapshot: React.Dispatch<React.SetStateAction<MapServerSnapshot>>;
    setCar: React.Dispatch<React.SetStateAction<CarState | null>>;
    setTimestamp: React.Dispatch<React.SetStateAction<number>>;
    localPlannerDt: number;
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
    setGlobalPlannerSegments,
    setLocalTrajectory,
    setReferencePoints,
    setMapSnapshot,
    setCar,
    setTimestamp,
    localPlannerDt,
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
    }, []); // eslint-disable-line react-hooks/exhaustive-deps

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

            void setLocalPlannerState(event.state, event.timestamp, localPlannerDt, localPlannerUpdateIntervalMs).catch(
                (error) => {
                    console.error('Failed to update local planner state', error);
                },
            );

            carRef.current = event.state;
            timestampRef.current = event.timestamp;
            setTimestamp(event.timestamp);
            setCar(event.state);
        });

        void (async () => {
            try {
                const mapServerNode = mapServerNodeRef.current;
                if (!mapServerNode) {
                    return;
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
            void stopSimulation().catch(() => {});
            resetComputeWorker('App unmounted');
        };
    }, []); // eslint-disable-line react-hooks/exhaustive-deps
}

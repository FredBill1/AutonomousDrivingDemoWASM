import { useEffect, useLayoutEffect, useRef, useState } from 'react';

import { AutoShrinkHeading } from './components/AutoShrinkHeading';
import { HistoryChart } from './components/HistoryChart';
import { MapViewport } from './components/MapViewport';
import { HISTORY_LIMIT, type CarState, type Mode } from './lib/appModel';
import { MapServerNode, flattenObstacleCoordinates } from './lib/mapServerNode';
import {
    checkCollision,
    checkTrajectoryCollision,
    ensureWasmCore,
    getCarConfigSnapshot,
    type HybridAStarProgress,
    type LocalPlannerPathPoint,
    type LocalPlannerReferencePoint,
    type LocalPlannerTrajectoryPoint,
} from './lib/wasmCore';
import { TrajectoryCollisionCheckingNode } from './lib/trajectoryCollisionCheckingNode';
import {
    FALLBACK_MAP_BOUNDING_BOX,
    DEFAULT_CAR_SHAPE,
    DEFAULT_MOTION_LIMITS,
    STACKED_LAYOUT_MAX_WIDTH_PX,
    STACKED_LAYOUT_MIN_MAP_HEIGHT_PX,
    STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX,
    STACKED_LAYOUT_GAP_PX,
    type CarShape,
    type DashboardLayout,
    type DragStartState,
    type GoalUnreachableState,
    type MapServerSnapshot,
    type MotionLimits,
} from './lib/appTypes';
import {
    createCarShape,
    createMotionLimits,
    formatFixedWithoutNegativeZero,
    toHybridAStarStartSeed,
    toTrajectoryPath,
} from './lib/appHelpers';
import { useSimulationSetup } from './hooks/useSimulationSetup';
import { usePlanningCallbacks } from './hooks/usePlanningCallbacks';

export type { CarShape, MotionLimits, GoalUnreachableState };

const LOCAL_PLANNER_UPDATE_INTERVAL_MS = 100;
const LOCAL_PLANNER_DT = 0.07;
const REPLAN_MAX_SPEED = 5 / 3.6;
const MAX_GLOBAL_PLANNER_DISPLAY_BATCHES = 32;

function App() {
    const [mode, setMode] = useState<Mode>('goal');
    const [timestamp, setTimestamp] = useState(0);
    const [mapSnapshot, setMapSnapshot] = useState<MapServerSnapshot>({
        boundingBox: FALLBACK_MAP_BOUNDING_BOX,
        knownObstacles: [],
        unknownObstacles: [],
    });
    const [carShape, setCarShape] = useState<CarShape>(DEFAULT_CAR_SHAPE);
    const [motionLimits, setMotionLimits] = useState<MotionLimits>(DEFAULT_MOTION_LIMITS);
    const [car, setCar] = useState<CarState | null>(null);
    const [goal, setGoal] = useState<CarState | null>(null);
    const [pressedPose, setPressedPose] = useState<CarState | null>(null);
    const [goalUnreachable, setGoalUnreachable] = useState<GoalUnreachableState>({
        visible: false,
        x: 0,
        y: 0,
    });
    const [globalTrajectory, setGlobalTrajectory] = useState<LocalPlannerTrajectoryPoint[] | null>(null);
    const [localTrajectory, setLocalTrajectory] = useState<LocalPlannerPathPoint[]>([]);
    const [referencePoints, setReferencePoints] = useState<LocalPlannerReferencePoint[]>([]);
    const [globalPlannerSegments, setGlobalPlannerSegments] = useState<HybridAStarProgress['segments'][]>([]);
    const [velocityHistory, setVelocityHistory] = useState([{ t: 0, value: 0 }]);
    const [steerHistory, setSteerHistory] = useState([{ t: 0, value: 0 }]);
    const [dashboardLayout, setDashboardLayout] = useState<DashboardLayout>('split');

    const mapServerNodeRef = useRef<MapServerNode | null>(null);
    const carRef = useRef<CarState | null>(car);
    const timestampRef = useRef(timestamp);
    const goalRef = useRef<CarState | null>(goal);
    const mapSnapshotRef = useRef(mapSnapshot);
    const globalTrajectoryRef = useRef<LocalPlannerTrajectoryPoint[] | null>(globalTrajectory);
    const localPlanningRef = useRef(false);
    const brakeTrajectoryRef = useRef<LocalPlannerReferencePoint[] | null>(null);
    const planningRequestRef = useRef(0);
    const dragStartRef = useRef<DragStartState | null>(null);
    const trajectoryCollisionCheckingNodeRef = useRef<TrajectoryCollisionCheckingNode | null>(null);
    const dashboardGridRef = useRef<HTMLElement | null>(null);

    if (trajectoryCollisionCheckingNodeRef.current === null) {
        trajectoryCollisionCheckingNodeRef.current = new TrajectoryCollisionCheckingNode(checkTrajectoryCollision);
    }

    if (mapServerNodeRef.current === null) {
        mapServerNodeRef.current = new MapServerNode(checkCollision, {
            backToCenter: DEFAULT_CAR_SHAPE.backToCenter,
            scanRadius: DEFAULT_MOTION_LIMITS.scanRadius,
        });
    }

    useEffect(() => {
        carRef.current = car;
    }, [car]);

    useEffect(() => {
        timestampRef.current = timestamp;
    }, [timestamp]);

    useEffect(() => {
        goalRef.current = goal;
    }, [goal]);

    useEffect(() => {
        mapSnapshotRef.current = mapSnapshot;
    }, [mapSnapshot]);

    useEffect(() => {
        globalTrajectoryRef.current = globalTrajectory;
    }, [globalTrajectory]);

    useLayoutEffect(() => {
        const host = dashboardGridRef.current;
        if (!host) {
            return;
        }

        const updateLayout = () => {
            const width = host.clientWidth;
            const height = host.clientHeight;
            const canStack =
                width <= STACKED_LAYOUT_MAX_WIDTH_PX &&
                height >=
                    STACKED_LAYOUT_MIN_MAP_HEIGHT_PX + STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX + STACKED_LAYOUT_GAP_PX;

            setDashboardLayout(canStack ? 'stacked' : 'split');
        };

        updateLayout();

        const resizeObserver = new ResizeObserver(updateLayout);
        resizeObserver.observe(host);

        return () => {
            resizeObserver.disconnect();
        };
    }, []);

    useEffect(() => {
        let active = true;

        void ensureWasmCore()
            .then(() => getCarConfigSnapshot())
            .then((snapshot) => {
                if (!active) {
                    return;
                }

                const nextCarShape = createCarShape(snapshot);
                const nextMotionLimits = createMotionLimits(snapshot);
                setCarShape(nextCarShape);
                setMotionLimits(nextMotionLimits);
                mapServerNodeRef.current?.setConfig({
                    backToCenter: snapshot.backToCenter,
                    scanRadius: snapshot.scanRadius,
                });
            })
            .catch((error) => {
                console.error('Failed to initialize WASM core', error);
            });

        return () => {
            active = false;
        };
    }, []);

    useSimulationSetup({
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
        localPlannerDt: LOCAL_PLANNER_DT,
        localPlannerUpdateIntervalMs: LOCAL_PLANNER_UPDATE_INTERVAL_MS,
        maxGlobalPlannerDisplayBatches: MAX_GLOBAL_PLANNER_DISPLAY_BATCHES,
    });

    useEffect(() => {
        if (!car) {
            return;
        }

        const mapUpdate = mapServerNodeRef.current?.update(car);
        if (!mapUpdate || mapUpdate.newObstacles.length === 0) {
            return;
        }

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
    }, [car]);

    useEffect(() => {
        if (!car) {
            return;
        }

        setVelocityHistory((history) => [
            ...history.slice(-HISTORY_LIMIT + 1),
            { t: timestamp, value: car.velocity * 3.6 },
        ]);
        setSteerHistory((history) => [
            ...history.slice(-HISTORY_LIMIT + 1),
            { t: timestamp, value: (car.steer * 180) / Math.PI },
        ]);
    }, [car, timestamp]);

    const {
        handleCancel,
        handleBrake,
        handleRestart,
        handleMapPrimaryDragStart,
        handleMapPrimaryDragMove,
        handleMapPrimaryDragEnd,
        handleMapPrimaryDragCancel,
    } = usePlanningCallbacks({
        mode,
        carRef,
        goalRef,
        mapSnapshotRef,
        globalTrajectoryRef,
        brakeTrajectoryRef,
        dragStartRef,
        planningRequestRef,
        localPlanningRef,
        trajectoryCollisionCheckingNodeRef,
        mapServerNodeRef,
        setCar,
        setGoal,
        setPressedPose,
        setGoalUnreachable,
        setGlobalTrajectory,
        setGlobalPlannerSegments,
        setLocalTrajectory,
        setReferencePoints,
        setMapSnapshot,
        replanMaxSpeed: REPLAN_MAX_SPEED,
        toHybridAStarStartSeed,
    });

    useEffect(() => {
        const handleKeyDown = (event: KeyboardEvent) => {
            if (event.repeat) {
                return;
            }

            switch (event.key.toLowerCase()) {
                case 'a':
                    event.preventDefault();
                    setMode('goal');
                    break;
                case 's':
                    event.preventDefault();
                    setMode('pose');
                    break;
                case 'd':
                    event.preventDefault();
                    void handleBrake();
                    break;
                case 'f':
                    event.preventDefault();
                    void handleCancel();
                    break;
                case 'r':
                    event.preventDefault();
                    void handleRestart();
                    break;
                default:
                    break;
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [handleBrake, handleCancel, handleRestart]);

    return (
        <div className="app-shell">
            <main ref={dashboardGridRef} className={`dashboard-grid dashboard-grid--${dashboardLayout}`}>
                <section className="panel panel-map">
                    <div className="panel-heading">
                        <h2>Visualization</h2>
                        <span className="panel-status">Timestamp: {timestamp.toFixed(1)}s</span>
                    </div>

                    <MapViewport
                        bounds={mapSnapshot.boundingBox}
                        mode={mode}
                        carShape={carShape}
                        motionLimits={motionLimits}
                        knownObstacles={mapSnapshot.knownObstacles}
                        unknownObstacles={mapSnapshot.unknownObstacles}
                        car={car}
                        goal={goal}
                        pressedPose={pressedPose}
                        goalUnreachable={goalUnreachable}
                        globalTrajectory={globalTrajectory ? toTrajectoryPath(globalTrajectory) : null}
                        localTrajectory={localTrajectory}
                        referencePoints={referencePoints}
                        globalPlannerSegments={globalPlannerSegments}
                        onPrimaryDragStart={handleMapPrimaryDragStart}
                        onPrimaryDragMove={handleMapPrimaryDragMove}
                        onPrimaryDragEnd={handleMapPrimaryDragEnd}
                        onPrimaryDragCancel={handleMapPrimaryDragCancel}
                    />
                </section>

                <div className={`side-stack side-stack--${dashboardLayout}`}>
                    <section className="panel chart-panel">
                        <div className="panel-heading compact">
                            <AutoShrinkHeading
                                text={`Velocity: ${formatFixedWithoutNegativeZero((car?.velocity ?? 0) * 3.6, 1)}km/h`}
                            />
                        </div>
                        <HistoryChart
                            points={velocityHistory}
                            minValue={motionLimits.minSpeedKmh}
                            maxValue={motionLimits.maxSpeedKmh}
                            lineColor={0x9fe870}
                        />
                    </section>

                    <section className="panel chart-panel">
                        <div className="panel-heading compact">
                            <AutoShrinkHeading
                                text={`Steer: ${formatFixedWithoutNegativeZero(((car?.steer ?? 0) * 180) / Math.PI, 1)}°`}
                            />
                        </div>
                        <HistoryChart
                            points={steerHistory}
                            minValue={-motionLimits.maxSteerDeg}
                            maxValue={motionLimits.maxSteerDeg}
                            lineColor={0x57d8ff}
                        />
                    </section>
                </div>
            </main>

            <section className="control-ribbon">
                <div className="segmented-control">
                    <button className={mode === 'goal' ? 'active' : ''} onClick={() => setMode('goal')}>
                        Set Goal(A)
                    </button>
                    <button className={mode === 'pose' ? 'active' : ''} onClick={() => setMode('pose')}>
                        Set Pose(S)
                    </button>
                </div>
                <button className="ghost-button" onClick={() => void handleBrake()}>
                    Brake(D)
                </button>
                <button className="ghost-button" onClick={() => void handleCancel()}>
                    Cancel(F)
                </button>
                <button className="accent-button" onClick={() => void handleRestart()}>
                    Restart(R)
                </button>
            </section>
        </div>
    );
}
export default App;

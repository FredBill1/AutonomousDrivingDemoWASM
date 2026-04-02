import { AutoShrinkHeading } from './components/AutoShrinkHeading';
import { HistoryChart } from './components/HistoryChart';
import { MapViewport } from './components/MapViewport';
import { usePlanningCallbacks } from './hooks/usePlanningCallbacks';
import { useAppState } from './hooks/useAppState';
import { useDashboardLayout } from './hooks/useDashboardLayout';
import { useKeyboardShortcuts } from './hooks/useKeyboardShortcuts';
import { useSimulationSetup } from './hooks/useSimulationSetup';
import { formatFixedWithoutNegativeZero, toHybridAStarStartSeed, toTrajectoryPath } from './lib/appHelpers';
import { HISTORY_LIMIT } from './lib/appModel';
import {
  type CarShape,
  type GoalUnreachableState,
  type MotionLimits,
} from './lib/appTypes';
import {
  LOCAL_PLANNER_UPDATE_INTERVAL_MS,
  MAX_GLOBAL_PLANNER_DISPLAY_BATCHES,
  MS_TO_KMH,
  RAD_TO_DEG,
  REPLAN_MAX_SPEED_MS,
} from './lib/constants';

export type { CarShape, GoalUnreachableState, MotionLimits };

type ChartPanelProps = {
  heading: string;
  points: { t: number; value: number }[];
  minValue: number;
  maxValue: number;
  lineColor: number;
};

function ChartPanel({ heading, points, minValue, maxValue, lineColor }: ChartPanelProps) {
  return (
    <section className="panel chart-panel">
      <div className="panel-heading compact">
        <AutoShrinkHeading text={heading} />
      </div>
      <HistoryChart points={points} minValue={minValue} maxValue={maxValue} lineColor={lineColor} />
    </section>
  );
}

function App() {
  const { state, refs, setters, dashboardGridRef } = useAppState();
  const dashboardLayout = useDashboardLayout(dashboardGridRef);

  useSimulationSetup({
    refs,
    setters,
    historyLimit: HISTORY_LIMIT,
    localPlannerUpdateIntervalMs: LOCAL_PLANNER_UPDATE_INTERVAL_MS,
    maxGlobalPlannerDisplayBatches: MAX_GLOBAL_PLANNER_DISPLAY_BATCHES,
  });

  const {
    handleCancel,
    handleBrake,
    handleRestart,
    handleMapPrimaryDragStart,
    handleMapPrimaryDragMove,
    handleMapPrimaryDragEnd,
    handleMapPrimaryDragCancel,
  } = usePlanningCallbacks({
    mode: state.mode,
    refs,
    setters,
    replanMaxSpeed: REPLAN_MAX_SPEED_MS,
    toHybridAStarStartSeed,
  });

  useKeyboardShortcuts({
    setMode: setters.setMode,
    handleBrake,
    handleCancel,
    handleRestart,
  });

  return (
    <div className="app-shell">
      <main ref={dashboardGridRef} className={`dashboard-grid dashboard-grid--${dashboardLayout}`}>
        <section className="panel panel-map">
          <div className="panel-heading">
            <h2>Visualization</h2>
            <span className="panel-status">Timestamp: {state.timestamp.toFixed(1)}s</span>
          </div>

          <MapViewport
            bounds={state.mapSnapshot.boundingBox}
            mode={state.mode}
            carShape={state.carShape}
            motionLimits={state.motionLimits}
            knownObstacles={state.mapSnapshot.knownObstacles}
            unknownObstacles={state.mapSnapshot.unknownObstacles}
            car={state.car}
            goal={state.goal}
            pressedPose={state.pressedPose}
            goalUnreachable={state.goalUnreachable}
            globalTrajectory={state.globalTrajectory ? toTrajectoryPath(state.globalTrajectory) : null}
            localTrajectory={state.localTrajectory}
            referencePoints={state.referencePoints}
            globalPlannerSegments={state.globalPlannerSegments}
            onPrimaryDragStart={handleMapPrimaryDragStart}
            onPrimaryDragMove={handleMapPrimaryDragMove}
            onPrimaryDragEnd={handleMapPrimaryDragEnd}
            onPrimaryDragCancel={handleMapPrimaryDragCancel}
          />
        </section>

        <div className={`side-stack side-stack--${dashboardLayout}`}>
          <ChartPanel
            heading={`Velocity: ${formatFixedWithoutNegativeZero((state.car?.velocity ?? 0) * MS_TO_KMH, 1)}km/h`}
            points={state.velocityHistory}
            minValue={state.motionLimits?.minSpeedKmh ?? 0}
            maxValue={state.motionLimits?.maxSpeedKmh ?? 0}
            lineColor={0x9fe870}
          />

          <ChartPanel
            heading={`Steer: ${formatFixedWithoutNegativeZero((state.car?.steer ?? 0) * RAD_TO_DEG, 1)}°`}
            points={state.steerHistory}
            minValue={-(state.motionLimits?.maxSteerDeg ?? 0)}
            maxValue={state.motionLimits?.maxSteerDeg ?? 0}
            lineColor={0x57d8ff}
          />
        </div>
      </main>

      <section className="control-ribbon">
        <div className="segmented-control">
          <button className={state.mode === 'goal' ? 'active' : ''} onClick={() => setters.setMode('goal')}>
            Set Goal(A)
          </button>
          <button className={state.mode === 'pose' ? 'active' : ''} onClick={() => setters.setMode('pose')}>
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

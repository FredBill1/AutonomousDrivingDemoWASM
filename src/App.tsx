import { useMemo, useState } from 'react';

import { AutoShrinkHeading } from './components/AutoShrinkHeading';
import { HistoryChart } from './components/HistoryChart';
import { MapViewport } from './components/MapViewport';
import { SettingsPanel } from './components/SettingsPanel';
import { useAppState } from './hooks/useAppState';
import { useDashboardLayout } from './hooks/useDashboardLayout';
import { useGlobalPlanning } from './hooks/useGlobalPlanning';
import { useKeyboardShortcuts } from './hooks/useKeyboardShortcuts';
import { usePlanningCommands } from './hooks/usePlanningCommands';
import { usePlanningDrag } from './hooks/usePlanningDrag';
import { useSimulationSetup } from './hooks/useSimulationSetup';
import { formatFixedWithoutNegativeZero, toHybridAStarStartSeed, toTrajectoryPath } from './lib/appHelpers';
import { cloneAppConfig, createDefaultAppConfig, loadStoredAppConfig, persistAppConfig } from './lib/appConfig';
import { HISTORY_LIMIT } from './lib/appModel';
import { type CarShape, type GoalUnreachableState, type MotionLimits } from './lib/appTypes';
import { KMH_TO_MS, MS_TO_KMH, RAD_TO_DEG } from './lib/constants';

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
  const [appConfig, setAppConfig] = useState(loadStoredAppConfig);
  const [settingsDraft, setSettingsDraft] = useState(createDefaultAppConfig);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [restartToken, setRestartToken] = useState(0);
  const { state, refs, updateState, dashboardGridRef } = useAppState();
  const dashboardLayout = useDashboardLayout(dashboardGridRef);
  const hasSettingsChanges = useMemo(
    () => JSON.stringify(settingsDraft) !== JSON.stringify(appConfig),
    [appConfig, settingsDraft],
  );

  useSimulationSetup({
    controllerConfig: appConfig.controller,
    refs,
    updateState,
    historyLimit: HISTORY_LIMIT,
    maxGlobalPlannerDisplayBatches: appConfig.ui.maxGlobalPlannerDisplayBatches,
    restartToken,
  });

  const { clearGlobalPlannerDisplaySegments, runGlobalPlan } = useGlobalPlanning({
    refs,
    updateState,
    replanMaxSpeed: appConfig.ui.replanMaxSpeedKmh * KMH_TO_MS,
    toHybridAStarStartSeed,
  });
  const { handleCancel, handleBrake, handleRestart } = usePlanningCommands({
    refs,
    updateState,
    clearGlobalPlannerDisplaySegments,
  });
  const { handleMapPrimaryDragStart, handleMapPrimaryDragMove, handleMapPrimaryDragEnd, handleMapPrimaryDragCancel } =
    usePlanningDrag({
      mode: state.mode,
      refs,
      updateState,
      handleBrake,
      handleCancel,
      runGlobalPlan,
    });

  useKeyboardShortcuts({
    setMode: (mode) => updateState('mode', mode),
    handleBrake,
    handleCancel,
    handleRestart,
  });

  const handleOpenSettings = () => {
    setSettingsDraft(cloneAppConfig(appConfig));
    setSettingsOpen(true);
  };

  const handleCloseSettings = () => {
    if (hasSettingsChanges) {
      setAppConfig(persistAppConfig(settingsDraft));
      setRestartToken((current) => current + 1);
    }
    setSettingsOpen(false);
  };

  return (
    <div className="app-shell">
      <main ref={dashboardGridRef} className={`dashboard-grid dashboard-grid--${dashboardLayout}`}>
        <section className="panel panel-map">
          <div className="panel-heading">
            <h2>Visualization</h2>
            <span className="panel-status">Timestamp: {state.timestamp.toFixed(1)}s</span>
          </div>

          <MapViewport
            scene={{
              bounds: state.mapSnapshot.boundingBox,
              mode: state.mode,
              carShape: state.carShape,
              motionLimits: state.motionLimits,
              knownObstacles: state.mapSnapshot.knownObstacles,
              unknownObstacles: state.mapSnapshot.unknownObstacles,
              car: state.car,
              goal: state.goal,
              pressedPose: state.pressedPose,
              goalUnreachable: state.goalUnreachable,
              globalTrajectory: state.globalTrajectory ? toTrajectoryPath(state.globalTrajectory) : null,
              localTrajectory: state.localTrajectory,
              referencePoints: state.referencePoints,
              globalPlannerSegments: state.globalPlannerSegments,
            }}
            interaction={{
              onPrimaryDragStart: handleMapPrimaryDragStart,
              onPrimaryDragMove: handleMapPrimaryDragMove,
              onPrimaryDragEnd: handleMapPrimaryDragEnd,
              onPrimaryDragCancel: handleMapPrimaryDragCancel,
            }}
            viewportConfig={appConfig.ui}
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
        <div className="control-ribbon__row">
          <div className="segmented-control">
            <button className={state.mode === 'goal' ? 'active' : ''} onClick={() => updateState('mode', 'goal')}>
              Set Goal(A)
            </button>
            <button className={state.mode === 'pose' ? 'active' : ''} onClick={() => updateState('mode', 'pose')}>
              Set Pose(S)
            </button>
          </div>
        </div>
        <div className="control-ribbon__row control-ribbon__row--actions">
          <button className="ghost-button" onClick={() => void handleBrake()}>
            Brake(D)
          </button>
          <button className="ghost-button" onClick={() => void handleCancel()}>
            Cancel(F)
          </button>
          <button className="accent-button" onClick={() => void handleRestart()}>
            Restart(R)
          </button>
          <button className="ghost-button" onClick={handleOpenSettings}>
            Settings
          </button>
        </div>
      </section>

      <SettingsPanel
        isOpen={settingsOpen}
        config={settingsDraft}
        hasChanges={hasSettingsChanges}
        onConfigChange={setSettingsDraft}
        onClose={handleCloseSettings}
        onReset={() => setSettingsDraft(createDefaultAppConfig())}
      />
    </div>
  );
}

export default App;

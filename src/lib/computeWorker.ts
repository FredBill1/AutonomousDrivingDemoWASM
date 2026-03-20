import initWasm, {
  CarConfig,
  CarState,
  HybridAStarPlanner,
  MpcReferenceTracker,
  mpc_control_preview,
  path_check_collision,
  trajectory_check_collision,
  rs_solve_path,
} from '../../wasm-core/pkg/wasm_core'

type WasmCarState = {
  x: number
  y: number
  yaw: number
  velocity: number
  steer: number
}

type WorkerRequest = {
  id: number
  type: string
  payload?: unknown
}

type WorkerResponse =
  | { id: number; ok: true; result: unknown }
  | { id: number; ok: false; error: string }

type WorkerEvent = {
  type: string
  payload?: unknown
}

type PlannerSession = {
  planner: HybridAStarPlanner
  cancelled: boolean
}

type HybridSeedPoint = {
  x: number
  y: number
  yaw: number
  velocity: number
}

type SimulationSession = {
  state: WasmCarState
  timestamp: number
  simDeltaTime: number
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> | null
  stopped: boolean
  simulationTimerId: ReturnType<typeof setTimeout> | null
  publishTimerId: ReturnType<typeof setInterval> | null
  loopToken: number
  stateVersion: number
}

type TrackingPlan = {
  path: Array<{ x: number; y: number; yaw: number }>
  directions: number[]
}

type LocalPlannerPathPoint = {
  x: number
  y: number
  yaw: number
}

type LocalPlannerReferencePoint = {
  x: number
  y: number
  velocity: number
  yaw: number
}

type LocalPlannerUpdateResult = {
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>
  localTrajectory: LocalPlannerPathPoint[]
  referencePoints: LocalPlannerReferencePoint[]
  brakeTrajectory: LocalPlannerReferencePoint[]
}

type LocalPlannerSession = {
  tracker: MpcReferenceTracker | null
  latestState: { state: WasmCarState; timestamp: number } | null
  simDeltaTime: number
  updateIntervalMs: number
  updateTimerId: number | null
  updateInFlight: boolean
}

let initPromise: Promise<CarConfig> | null = null
let nextPlannerToken = 1
let activePlanner: PlannerSession | null = null
let simulationSession: SimulationSession | null = null
let localPlannerSession: LocalPlannerSession | null = null

const HYBRID_STEP_BUDGET = 96
const HYBRID_SEGMENT_BATCH_SIZE = 320
const DEFAULT_SIM_DELTA_TIME = 0.015
const DEFAULT_SIM_INTERVAL_MS = 20
const DEFAULT_PUBLISH_INTERVAL_MS = 50
const DEFAULT_LOCAL_PLANNER_DT = 0.07
const DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS = 100

async function ensureWasmCore() {
  if (!initPromise) {
    initPromise = initWasm().then(() => new CarConfig())
  }
  return initPromise
}

function postEvent(type: string, payload?: unknown) {
  self.postMessage({ type, payload } satisfies WorkerEvent)
}

function emitSimulationState() {
  if (!simulationSession) return
  postEvent('simulationState', {
    timestamp: simulationSession.timestamp,
    state: simulationSession.state,
  })
}

function applySimulationStop(session: SimulationSession) {
  session.state = {
    ...session.state,
    velocity: 0,
    steer: 0,
  }
  session.controlSequence = null
  session.stopped = true
  session.stateVersion += 1
}

function cloneControlSequence(
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>,
) {
  return controlSequence.map((point) => ({
    timestamp: point.timestamp,
    targetVelocity: point.targetVelocity,
    targetSteer: point.targetSteer,
  }))
}

function assertValidControlSequence(
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>,
) {
  if (controlSequence.length < 2) {
    throw new Error('Control sequence must contain at least two control points')
  }

  for (let index = 0; index < controlSequence.length; index += 1) {
    const point = controlSequence[index]
    if (!Number.isFinite(point.timestamp) || !Number.isFinite(point.targetVelocity) || !Number.isFinite(point.targetSteer)) {
      throw new Error('Control sequence contains non-finite values')
    }
    if (index > 0 && !(controlSequence[index - 1].timestamp < point.timestamp)) {
      throw new Error('Control sequence timestamps must be strictly increasing')
    }
  }
}

function setSimulationControlSequenceInternal(
  session: SimulationSession,
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> | null,
) {
  if (session.stopped) {
    return
  }

  if (!controlSequence || controlSequence.length === 0) {
    session.controlSequence = null
    session.stateVersion += 1
    return
  }

  assertValidControlSequence(controlSequence)
  session.controlSequence = cloneControlSequence(controlSequence)
  session.stateVersion += 1
}

function sampleControl(
  controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> | null,
  timestamp: number,
) {
  if (!controlSequence || controlSequence.length === 0) {
    return null
  }

  if (timestamp <= controlSequence[0].timestamp) {
    return controlSequence[0]
  }

  const last = controlSequence[controlSequence.length - 1]
  if (timestamp >= last.timestamp) {
    return last
  }

  for (let index = 1; index < controlSequence.length; index += 1) {
    const next = controlSequence[index]
    if (timestamp > next.timestamp) {
      continue
    }

    const prev = controlSequence[index - 1]
    const ratio = (timestamp - prev.timestamp) / (next.timestamp - prev.timestamp)
    return {
      timestamp,
      targetVelocity: prev.targetVelocity + (next.targetVelocity - prev.targetVelocity) * ratio,
      targetSteer: prev.targetSteer + (next.targetSteer - prev.targetSteer) * ratio,
    }
  }

  return last
}

function clearSimulationTimers() {
  if (!simulationSession) return
  if (simulationSession.simulationTimerId !== null) {
    clearTimeout(simulationSession.simulationTimerId)
    simulationSession.simulationTimerId = null
  }
  if (simulationSession.publishTimerId !== null) {
    clearInterval(simulationSession.publishTimerId)
    simulationSession.publishTimerId = null
  }
}

async function computeStepCarState(current: WasmCarState, targetVelocity: number, targetSteer: number, dt: number) {
  const config = await ensureWasmCore()
  const state = new CarState(current.x, current.y, current.yaw, current.velocity, current.steer)
  try {
    const next = state.stepped(config, targetVelocity, targetSteer, dt)
    try {
      return {
        x: next.x,
        y: next.y,
        yaw: next.yaw,
        velocity: next.velocity,
        steer: next.steer,
      }
    } finally {
      next.free()
    }
  } finally {
    state.free()
  }
}

async function computeOpenLoopStepCarState(current: WasmCarState, dt: number) {
  return computeStepCarState(current, current.velocity, current.steer, dt)
}

async function advanceSimulation(session: SimulationSession) {
  const sampledControl = sampleControl(session.controlSequence, session.timestamp)
  if (!sampledControl) {
    return computeOpenLoopStepCarState(session.state, session.simDeltaTime)
  }
  return computeStepCarState(session.state, sampledControl.targetVelocity, sampledControl.targetSteer, session.simDeltaTime)
}

function scheduleSimulationTick(session: SimulationSession, simulationIntervalMs: number, loopToken: number) {
  session.simulationTimerId = setTimeout(() => {
    if (!simulationSession || simulationSession !== session || session.loopToken !== loopToken) {
      return
    }

    const stateVersion = session.stateVersion
    session.timestamp += session.simDeltaTime

    void advanceSimulation(session)
      .then((next) => {
        if (
          !simulationSession ||
          simulationSession !== session ||
          session.loopToken !== loopToken ||
          session.stateVersion !== stateVersion
        ) {
          return
        }
        session.state = next
      })
      .catch((error) => {
        console.error('Failed to advance simulation', error)
      })
      .finally(() => {
        if (!simulationSession || simulationSession !== session || session.loopToken !== loopToken) {
          return
        }
        scheduleSimulationTick(session, simulationIntervalMs, loopToken)
      })
  }, simulationIntervalMs)
}

async function startSimulationLoop(simulationIntervalMs: number, publishIntervalMs: number) {
  if (!simulationSession) return
  clearSimulationTimers()

  simulationSession.loopToken += 1
  const loopToken = simulationSession.loopToken
  scheduleSimulationTick(simulationSession, simulationIntervalMs, loopToken)

  simulationSession.publishTimerId = setInterval(() => {
    emitSimulationState()
  }, publishIntervalMs)
}

function buildTrajectoryCollisionInput(plan: TrackingPlan | null) {
  if (!plan || plan.path.length === 0) {
    return null
  }
  return Float64Array.from(plan.path.flatMap((point) => [point.x, point.y, point.yaw]))
}

function checkTrajectoryCollision(config: CarConfig, plan: TrackingPlan | null, obstacleCoordinates: ArrayLike<number>) {
  const trajectory = buildTrajectoryCollisionInput(plan)
  if (!trajectory || obstacleCoordinates.length < 2) {
    return false
  }
  return trajectory_check_collision(config, trajectory, Float64Array.from(obstacleCoordinates))
}

function decodeFlatCoordinates(flatCoordinates: Float64Array): Array<{ x: number; y: number; yaw: number }> {
  const points = []
  for (let index = 0; index < flatCoordinates.length; index += 3) {
    points.push({
      x: flatCoordinates[index],
      y: flatCoordinates[index + 1],
      yaw: flatCoordinates[index + 2],
    })
  }
  return points
}

function flattenTrajectoryPoints(points: Array<{ x: number; y: number; yaw: number; direction: number }>) {
  return points.flatMap((point) => [point.x, point.y, point.yaw, point.direction])
}

function decodePredictedStateQuads(flatValues: number[] | Float64Array): LocalPlannerPathPoint[] {
  const points: LocalPlannerPathPoint[] = []
  for (let index = 0; index < flatValues.length; index += 4) {
    points.push({
      x: flatValues[index],
      y: flatValues[index + 1],
      yaw: flatValues[index + 3],
    })
  }
  return points
}

function decodePlannerStateQuads(flatValues: number[] | Float64Array): LocalPlannerReferencePoint[] {
  const points: LocalPlannerReferencePoint[] = []
  for (let index = 0; index < flatValues.length; index += 4) {
    points.push({
      x: flatValues[index],
      y: flatValues[index + 1],
      yaw: flatValues[index + 2],
      velocity: flatValues[index + 3],
    })
  }
  return points
}

function decodeControlPairs(flatValues: number[] | Float64Array, timestamp: number, dt: number, initialVelocity: number) {
  const controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }> = []
  let velocity = initialVelocity
  for (let index = 0; index < flatValues.length; index += 2) {
    velocity += flatValues[index] * dt
    controlSequence.push({
      timestamp: timestamp + (index / 2) * dt,
      targetVelocity: velocity,
      targetSteer: flatValues[index + 1],
    })
  }
  return controlSequence
}

async function runLocalPlannerUpdate(
  tracker: MpcReferenceTracker,
  state: WasmCarState,
  timestamp: number,
  dt: number,
): Promise<LocalPlannerUpdateResult | null> {
  const referenceResult = tracker.update(state.x, state.y, state.yaw, state.velocity, dt)
  const referenceStates = referenceResult.reference_states
  const modelReferenceStates = referenceResult.model_reference_states
  if (referenceStates.length === 0) {
    referenceResult.free()
    return null
  }

  const brakeTrajectory = referenceResult.brake_trajectory
  const controlResult = mpc_control_preview(
    modelReferenceStates,
    state.x,
    state.y,
    state.velocity,
    state.yaw,
    state.steer,
    dt,
  )
  try {
    return {
      controlSequence: decodeControlPairs(controlResult.controls, timestamp, dt, state.velocity),
      localTrajectory: decodePredictedStateQuads(controlResult.predicted_states).map((point) => ({
        x: point.x,
        y: point.y,
        yaw: point.yaw,
      })),
      referencePoints: decodePlannerStateQuads(referenceStates),
      brakeTrajectory: decodePlannerStateQuads(brakeTrajectory),
    }
  } finally {
    controlResult.free()
    referenceResult.free()
  }
}

function ensureLocalPlannerSession() {
  if (!localPlannerSession) {
    localPlannerSession = {
      tracker: null,
      latestState: null,
      simDeltaTime: DEFAULT_LOCAL_PLANNER_DT,
      updateIntervalMs: DEFAULT_LOCAL_PLANNER_UPDATE_INTERVAL_MS,
      updateTimerId: null,
      updateInFlight: false,
    }
  }

  if (localPlannerSession.updateTimerId === null) {
    localPlannerSession.updateTimerId = setInterval(() => {
      if (!localPlannerSession || localPlannerSession.updateInFlight) {
        return
      }

      const tracker = localPlannerSession.tracker
      const latestState = localPlannerSession.latestState
      if (!tracker || !latestState) {
        return
      }

      const activeSession = localPlannerSession
      activeSession.updateInFlight = true
      void runLocalPlannerUpdate(tracker, latestState.state, latestState.timestamp, activeSession.simDeltaTime)
        .then((result) => {
          if (!result || !localPlannerSession || localPlannerSession !== activeSession) {
            return
          }

          if (simulationSession) {
            setSimulationControlSequenceInternal(simulationSession, result.controlSequence)
          }
          postEvent('localPlannerUpdate', result)
        })
        .catch((error) => {
          console.error('Failed to update local planner', error)
        })
        .finally(() => {
          if (localPlannerSession === activeSession) {
            activeSession.updateInFlight = false
          }
        })
    }, localPlannerSession.updateIntervalMs)
  }

  return localPlannerSession
}

function clearLocalPlannerTimer() {
  if (!localPlannerSession || localPlannerSession.updateTimerId === null) {
    return
  }
  clearInterval(localPlannerSession.updateTimerId)
  localPlannerSession.updateTimerId = null
}

function decodeHybridResult(result: {
  token: number
  flat_path: Float64Array | number[]
  explored_segments: Float64Array | number[]
  explored_count: number
  analytic_expansions: number
  success: boolean
}) {
  if (!result.success) {
    return null
  }

  const pathValues = result.flat_path
  const exploredValues = result.explored_segments

  const path: Array<{ x: number; y: number; yaw: number }> = []
  const directions: number[] = []
  for (let index = 0; index < pathValues.length; index += 4) {
    path.push({ x: pathValues[index], y: pathValues[index + 1], yaw: pathValues[index + 2] })
    directions.push(pathValues[index + 3])
  }

  const exploredSegments: Array<{ x1: number; y1: number; x2: number; y2: number }> = []
  for (let index = 0; index < exploredValues.length; index += 4) {
    exploredSegments.push({
      x1: exploredValues[index],
      y1: exploredValues[index + 1],
      x2: exploredValues[index + 2],
      y2: exploredValues[index + 3],
    })
  }

  return {
    token: result.token,
    path,
    directions,
    exploredSegments,
    exploredCount: result.explored_count,
    analyticExpansions: result.analytic_expansions,
  }
}

function snapshotHybridResult(
  token: number,
  result: {
    flat_path: Float64Array
    explored_segments: Float64Array
    explored_count: number
    analytic_expansions: number
    success: boolean
    free?: () => void
  },
) {
  try {
    return decodeHybridResult({
      token,
      flat_path: result.flat_path,
      explored_segments: result.explored_segments,
      explored_count: result.explored_count,
      analytic_expansions: result.analytic_expansions,
      success: result.success,
    })
  } finally {
    result.free?.()
  }
}

function flattenHybridSeedPoints(seed: HybridSeedPoint[]) {
  return seed.flatMap((point) => [point.x, point.y, point.yaw, point.velocity])
}

function decodeExploredSegments(flatSegments: Float64Array | number[]) {
  const segments: Array<{ x1: number; y1: number; x2: number; y2: number }> = []
  for (let index = 0; index < flatSegments.length; index += 4) {
    segments.push({
      x1: flatSegments[index],
      y1: flatSegments[index + 1],
      x2: flatSegments[index + 2],
      y2: flatSegments[index + 3],
    })
  }
  return segments
}

const handlers = {
  async getCarConfigSnapshot() {
    const config = await ensureWasmCore()
    return {
      wheelBase: config.wheel_base,
      length: config.length,
      width: config.width,
      backToWheel: config.back_to_wheel,
      wheelLength: config.wheel_length,
      wheelWidth: config.wheel_width,
      wheelSpacing: config.wheel_spacing,
      backToCenter: config.back_to_center,
      collisionLength: config.collision_length,
      collisionWidth: config.collision_width,
      collisionRadius: config.collision_radius,
      targetMaxSteer: config.target_max_steer,
      maxSteer: config.max_steer,
      maxSteerSpeed: config.max_steer_speed,
      maxSpeed: config.max_speed,
      minSpeed: config.min_speed,
      maxAccel: config.max_accel,
      maxCentripetalAccel: config.max_centripetal_accel,
      targetSpeed: config.target_speed,
      targetMinTurningRadius: config.target_min_turning_radius,
      scanRadius: config.scan_radius,
    }
  },

  async stepCarState(payload: { current: WasmCarState; targetVelocity: number; targetSteer: number; dt: number }) {
    const { current, targetVelocity, targetSteer, dt } = payload
    return computeStepCarState(current, targetVelocity, targetSteer, dt)
  },

  async initSimulation(payload: {
    state: WasmCarState
    timestamp?: number
    simDeltaTime?: number
    simulationIntervalMs?: number
    publishIntervalMs?: number
  }) {
    await ensureWasmCore()
    simulationSession = {
      state: payload.state,
      timestamp: payload.timestamp ?? 0,
      simDeltaTime: payload.simDeltaTime ?? DEFAULT_SIM_DELTA_TIME,
      controlSequence: null,
      stopped: true,
      simulationTimerId: null,
      publishTimerId: null,
      loopToken: 0,
      stateVersion: 0,
    }
    await startSimulationLoop(payload.simulationIntervalMs ?? DEFAULT_SIM_INTERVAL_MS, payload.publishIntervalMs ?? DEFAULT_PUBLISH_INTERVAL_MS)
    return null
  },

  async setSimulationState(payload: { state: WasmCarState; timestamp?: number }) {
    await ensureWasmCore()
    if (!simulationSession) {
      simulationSession = {
        state: payload.state,
        timestamp: payload.timestamp ?? 0,
        simDeltaTime: DEFAULT_SIM_DELTA_TIME,
        controlSequence: null,
        stopped: true,
        simulationTimerId: null,
        publishTimerId: null,
        loopToken: 0,
        stateVersion: 0,
      }
      await startSimulationLoop(DEFAULT_SIM_INTERVAL_MS, DEFAULT_PUBLISH_INTERVAL_MS)
    } else {
      simulationSession.state = payload.state
      simulationSession.controlSequence = null
      simulationSession.stopped = true
      simulationSession.stateVersion += 1
      if (payload.timestamp !== undefined) {
        simulationSession.timestamp = payload.timestamp
      }
    }
    return null
  },

  async setSimulationControlSequence(payload: {
    controlSequence: Array<{ timestamp: number; targetVelocity: number; targetSteer: number }>
  }) {
    if (!simulationSession) {
      throw new Error('Simulation not initialized')
    }
    setSimulationControlSequenceInternal(simulationSession, payload.controlSequence)
    return null
  },

  async stopSimulationMotion() {
    if (!simulationSession) {
      throw new Error('Simulation not initialized')
    }
    applySimulationStop(simulationSession)
    return null
  },

  async setLocalPlannerTrajectory(payload: {
    trajectory: Array<{ x: number; y: number; yaw: number; direction: number }> | null
  }) {
    const session = ensureLocalPlannerSession()

    if (!payload.trajectory || payload.trajectory.length === 0) {
      session.tracker?.brake()
      return null
    }

    session.tracker?.free()
    session.tracker = new MpcReferenceTracker(Float64Array.from(flattenTrajectoryPoints(payload.trajectory)))
    return null
  },

  async setLocalPlannerState(payload: { state: WasmCarState; timestamp: number; dt?: number; updateIntervalMs?: number }) {
    const session = ensureLocalPlannerSession()
    session.latestState = {
      state: payload.state,
      timestamp: payload.timestamp,
    }
    if (payload.dt !== undefined) {
      session.simDeltaTime = payload.dt
    }
    if (payload.updateIntervalMs !== undefined && payload.updateIntervalMs !== session.updateIntervalMs) {
      session.updateIntervalMs = payload.updateIntervalMs
      clearLocalPlannerTimer()
      ensureLocalPlannerSession()
    }
    return null
  },

  async brakeLocalPlanner() {
    localPlannerSession?.tracker?.brake()
    return null
  },

  async cancelLocalPlanner() {
    if (localPlannerSession) {
      localPlannerSession.tracker?.free()
      localPlannerSession.tracker = null
    }
    return null
  },

  async resumeSimulationMotion() {
    if (!simulationSession) {
      throw new Error('Simulation not initialized')
    }
    simulationSession.stopped = false
    return null
  },

  async stopSimulation() {
    clearSimulationTimers()
    simulationSession = null
    clearLocalPlannerTimer()
    localPlannerSession?.tracker?.free()
    localPlannerSession = null
    return null
  },

  async checkCollision(payload: { state: WasmCarState; obstacleCoordinates: number[] }) {
    const config = await ensureWasmCore()
    const { state: stateLike, obstacleCoordinates } = payload
    const state = new CarState(stateLike.x, stateLike.y, stateLike.yaw, stateLike.velocity, stateLike.steer)
    try {
      return state.check_collision(config, Float64Array.from(obstacleCoordinates))
    } finally {
      state.free()
    }
  },

  async checkPathCollision(payload: { path: Array<{ x: number; y: number; yaw: number }>; obstacleCoordinates: number[] }) {
    const config = await ensureWasmCore()
    const flatPath = payload.path.flatMap((point) => [point.x, point.y, point.yaw])
    return path_check_collision(config, Float64Array.from(flatPath), Float64Array.from(payload.obstacleCoordinates))
  },

  async checkTrajectoryCollision(payload: { path: Array<{ x: number; y: number; yaw: number }>; obstacleCoordinates: number[] }) {
    const config = await ensureWasmCore()
    return checkTrajectoryCollision(config, { path: payload.path, directions: [] }, payload.obstacleCoordinates)
  },

  async solveReedsSheppCandidates(payload: {
    start: WasmCarState
    goal: WasmCarState
    turnRadii: number[]
    runwayLengths: number[]
    stepSize: number
    lengthTolerance: number
  }) {
    await ensureWasmCore()

    const solutions: Array<{
      path: Array<{ x: number; y: number; yaw: number }>
      totalLength: number
      segmentCount: number
      runwayLength: number
      turnRadius: number
    }> = []

    for (const turnRadius of payload.turnRadii) {
      for (const runwayLength of payload.runwayLengths) {
        try {
          const solvedPath = rs_solve_path(
            payload.start.x,
            payload.start.y,
            payload.start.yaw,
            payload.goal.x,
            payload.goal.y,
            payload.goal.yaw,
            turnRadius,
            runwayLength,
            payload.stepSize,
            payload.lengthTolerance,
          )

          try {
            solutions.push({
              path: decodeFlatCoordinates(solvedPath.flat_coordinates()),
              totalLength: solvedPath.total_length(),
              segmentCount: solvedPath.segment_count(),
              runwayLength: solvedPath.runway_length(),
              turnRadius: solvedPath.turn_radius(),
            })
          } finally {
            solvedPath.free()
          }
        } catch {
          // Ignore invalid combinations.
        }
      }
    }

    solutions.sort((left, right) => {
      if (Math.abs(left.totalLength - right.totalLength) < payload.lengthTolerance) {
        return left.segmentCount - right.segmentCount
      }
      return left.totalLength - right.totalLength
    })

    return solutions
  },

  async solveHybridAStar(payload: {
    start: WasmCarState | HybridSeedPoint[]
    startIsTrajectorySeed?: boolean
    goal: WasmCarState
    obstacleCoordinates: number[]
    maxIterations: number
    requestToken?: number
  }) {
    await ensureWasmCore()

    activePlanner?.planner.free()
    const planner = payload.startIsTrajectorySeed
      ? HybridAStarPlanner.from_trajectory_seed(
          Float64Array.from(flattenHybridSeedPoints(payload.start as HybridSeedPoint[])),
          payload.goal.x,
          payload.goal.y,
          payload.goal.yaw,
          Float64Array.from(payload.obstacleCoordinates),
          payload.maxIterations,
        )
      : new HybridAStarPlanner(
          (payload.start as WasmCarState).x,
          (payload.start as WasmCarState).y,
          (payload.start as WasmCarState).yaw,
          payload.goal.x,
          payload.goal.y,
          payload.goal.yaw,
          Float64Array.from(payload.obstacleCoordinates),
          payload.maxIterations,
        )
    activePlanner = {
      planner,
      cancelled: false,
    }

    const plannerToken = payload.requestToken ?? nextPlannerToken
    if (payload.requestToken === undefined) {
      nextPlannerToken += 1
    }
    const plannerSession = activePlanner

    while (!plannerSession.cancelled) {
      const finished = plannerSession.planner.step(HYBRID_STEP_BUDGET)
      const exploredFlat = plannerSession.planner.take_explored_segments()
      if (exploredFlat.length > 0) {
        const decoded = decodeExploredSegments(exploredFlat)
        for (let index = 0; index < decoded.length; index += HYBRID_SEGMENT_BATCH_SIZE) {
          postEvent('hybridAStarProgress', {
            token: plannerToken,
            segments: decoded.slice(index, index + HYBRID_SEGMENT_BATCH_SIZE),
            exploredCount: plannerSession.planner.explored_count,
            analyticExpansions: plannerSession.planner.analytic_expansions,
          })
        }
      }

      if (finished) {
        const result = plannerSession.planner.take_result()
        plannerSession.planner.free()
        activePlanner = null
        if (!result) {
          return null
        }
        return snapshotHybridResult(plannerToken, result)
      }

      await new Promise<void>((resolve) => setTimeout(resolve, 0))
    }

    plannerSession.planner.free()
    activePlanner = null
    throw new Error('Hybrid A* search cancelled')
  },

  async cancelHybridAStar() {
    if (activePlanner) {
      activePlanner.cancelled = true
    }
    return null
  },
} as const

self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const message = event.data
  const handler = handlers[message.type as keyof typeof handlers] as ((payload: never) => Promise<unknown>) | undefined

  if (!handler) {
    self.postMessage({ id: message.id, ok: false, error: `Unknown worker request: ${message.type}` } satisfies WorkerResponse)
    return
  }

  void handler(message.payload as never)
    .then((result) => {
      self.postMessage({ id: message.id, ok: true, result } satisfies WorkerResponse)
    })
    .catch((error: unknown) => {
      const errorMessage = error instanceof Error ? error.message : String(error)
      self.postMessage({ id: message.id, ok: false, error: errorMessage } satisfies WorkerResponse)
    })
}

export {}

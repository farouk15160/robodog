import { useLocalSearchParams, useRouter } from 'expo-router';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  ActivityIndicator,
  AppState,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  useWindowDimensions,
  View,
} from 'react-native';
import { SafeAreaView, useSafeAreaInsets } from 'react-native-safe-area-context';
import { Gesture } from 'react-native-gesture-handler';
import { WebView } from 'react-native-webview';

import { Joystick } from '../components/Joystick';
import {
  buildEstopEnvelope,
  buildGaitEnvelope,
  buildGreetingEnvelope,
  buildVelocityEnvelope,
  parseBridgeFeedback,
  serializeControlEnvelope,
} from '../domain/controlBridge';
import type { ControlEnvelope, ControlLimits, JsonObject } from '../domain/controlBridge';
import {
  commandForHandheldInput,
  clampJoystick,
  createHandheldInput,
  isHandheldInputActive,
  joystickGeometry,
  releaseHandheldInput,
  setHandheldStickActive,
  updateHandheldStick,
} from '../domain/joystick';
import type { HandheldStick, JoystickVector } from '../domain/joystick';
import { resolveControllerLayout } from '../domain/layout';
import {
  buildEndpointUrl,
  buildRemoteControlUrl,
  isAllowedRobotNavigation,
  parseRobotState,
} from '../domain/robot';
import type { RobotDescriptor, RobotState } from '../domain/robot';
import { connectManualRobot, fetchRobotState } from '../services/robotClient';
import { colors } from '../theme';

type ViewMode = 'cockpit' | 'web';

const DEFAULT_LIMITS: ControlLimits = Object.freeze({ linear: 0.5, angular: 1.2 });
const ZERO_STICK: JoystickVector = Object.freeze({ x: 0, y: 0 });
const LANDSCAPE_HIT_SLOP = Object.freeze({ top: 4, bottom: 4 });

const numberField = (source: JsonObject, key: string, fallback: number): number => {
  const value = source[key];
  return typeof value === 'number' && Number.isFinite(value) && value > 0 ? value : fallback;
};

const bridgeUrlFor = (remoteUrl: string): string => {
  const url = new URL(remoteUrl);
  url.searchParams.set('native_bridge', '1');
  return url.toString();
};

interface MetricProps {
  readonly label: string;
  readonly value: string;
  readonly tone?: string;
}

function Metric({ label, value, tone = colors.text }: MetricProps) {
  return <View style={styles.metric}>
    <Text style={styles.metricLabel}>{label}</Text>
    <Text numberOfLines={1} style={[styles.metricValue, { color: tone }]}>{value}</Text>
  </View>;
}

export default function ControlScreen() {
  const params = useLocalSearchParams<{ origin?: string }>();
  const router = useRouter();
  const { width, height } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const layout = useMemo(() => resolveControllerLayout(
    width - insets.left - insets.right,
    height - insets.top - insets.bottom,
  ), [height, insets.bottom, insets.left, insets.right, insets.top, width]);
  const bridgeRef = useRef<WebView>(null);
  const commandSequence = useRef(0);
  const bridgeReadyRef = useRef(false);
  const inputRef = useRef(createHandheldInput());
  const limitsRef = useRef<ControlLimits>(DEFAULT_LIMITS);

  const [robot, setRobot] = useState<RobotDescriptor | null>(null);
  const [state, setState] = useState<RobotState | null>(null);
  const [error, setError] = useState('');
  const [view, setView] = useState<ViewMode>('cockpit');
  const [bridgeReady, setBridgeReady] = useState(false);
  const [bridgeConnected, setBridgeConnected] = useState(false);
  const [bridgeStatus, setBridgeStatus] = useState('Starting control link…');
  const [deadmanHeld, setDeadmanHeld] = useState(false);
  const [moveVisual, setMoveVisual] = useState<JoystickVector>(ZERO_STICK);
  const [turnVisual, setTurnVisual] = useState<JoystickVector>(ZERO_STICK);
  const [movePressed, setMovePressed] = useState(false);
  const [turnPressed, setTurnPressed] = useState(false);
  const controlAvailable = bridgeReady && bridgeConnected && Boolean(state) && !state?.safety.estop;
  const controlAvailableRef = useRef(controlAvailable);
  controlAvailableRef.current = controlAvailable;

  const nextCommandId = useCallback((kind: string) => {
    commandSequence.current += 1;
    return `${kind}-${commandSequence.current}`;
  }, []);

  const postEnvelope = useCallback((envelope: ControlEnvelope): boolean => {
    if (!bridgeReadyRef.current || !bridgeRef.current) return false;
    bridgeRef.current.postMessage(serializeControlEnvelope(envelope));
    return true;
  }, []);

  const sendVelocity = useCallback((held = isHandheldInputActive(inputRef.current)) => {
    const input = inputRef.current;
    const command = commandForHandheldInput(
      held ? input : releaseHandheldInput(input),
      controlAvailableRef.current,
      limitsRef.current,
    );
    postEnvelope(buildVelocityEnvelope(nextCommandId('motion'), command, limitsRef.current));
  }, [nextCommandId, postEnvelope]);

  const releaseMotion = useCallback(() => {
    const wasHeld = isHandheldInputActive(inputRef.current);
    inputRef.current = releaseHandheldInput(inputRef.current);
    setDeadmanHeld(false);
    setMoveVisual(ZERO_STICK);
    setTurnVisual(ZERO_STICK);
    setMovePressed(false);
    setTurnPressed(false);
    if (wasHeld || bridgeReadyRef.current) sendVelocity(false);
  }, [sendVelocity]);

  const joystickTravel = useMemo(
    () => joystickGeometry(layout.joystickDiameter).travel,
    [layout.joystickDiameter],
  );
  const joystickGestures = useMemo(() => {
    const updateVector = (stick: HandheldStick, value: JoystickVector) => {
      inputRef.current = updateHandheldStick(inputRef.current, stick, value);
      if (isHandheldInputActive(inputRef.current)) sendVelocity(true);
    };
    const setActive = (stick: HandheldStick, active: boolean) => {
      inputRef.current = setHandheldStickActive(inputRef.current, stick, active);
      const driving = isHandheldInputActive(inputRef.current) && controlAvailableRef.current;
      setDeadmanHeld(driving);
      sendVelocity(driving);
    };
    const move = Gesture.Pan()
      .enabled(controlAvailable)
      .minDistance(0)
      .shouldCancelWhenOutside(false)
      .runOnJS(true)
      .onBegin(() => { setMovePressed(true); setActive('move', true); })
      .onUpdate((event) => {
        const next = clampJoystick({
          x: event.translationX / joystickTravel,
          y: event.translationY / joystickTravel,
        });
        setMoveVisual(next);
        updateVector('move', next);
      })
      .onFinalize(() => {
        setMovePressed(false);
        setMoveVisual(ZERO_STICK);
        updateVector('move', ZERO_STICK);
        setActive('move', false);
      });
    const turn = Gesture.Pan()
      .enabled(controlAvailable)
      .minDistance(0)
      .shouldCancelWhenOutside(false)
      .runOnJS(true)
      .onBegin(() => { setTurnPressed(true); setActive('turn', true); })
      .onUpdate((event) => {
        const next = clampJoystick({
          x: event.translationX / joystickTravel,
          y: event.translationY / joystickTravel,
        });
        setTurnVisual(next);
        updateVector('turn', next);
      })
      .onFinalize(() => {
        setTurnPressed(false);
        setTurnVisual(ZERO_STICK);
        updateVector('turn', ZERO_STICK);
        setActive('turn', false);
      });
    move.simultaneousWithExternalGesture(turn);
    turn.simultaneousWithExternalGesture(move);
    return Object.freeze({ move, turn });
  }, [controlAvailable, joystickTravel, sendVelocity]);

  useEffect(() => {
    let active = true;
    if (!params.origin) {
      setError('Missing robot address');
      return () => { active = false; };
    }
    connectManualRobot(params.origin)
      .then((value) => { if (active) setRobot(value); })
      .catch((reason) => {
        if (active) setError(reason instanceof Error ? reason.message : 'Connection failed');
      });
    return () => { active = false; };
  }, [params.origin]);

  useEffect(() => {
    if (!robot) return undefined;
    let active = true;
    const refresh = async () => {
      if (AppState.currentState !== 'active') return;
      try {
        const next = await fetchRobotState(robot);
        if (active) { setState(next); setError(''); }
      } catch (reason) {
        if (active && !bridgeConnected) {
          setError(reason instanceof Error ? reason.message : 'Telemetry unavailable');
        }
      }
    };
    void refresh();
    const timer = setInterval(refresh, 1000);
    return () => { active = false; clearInterval(timer); };
  }, [bridgeConnected, robot]);

  useEffect(() => {
    if (!deadmanHeld) return undefined;
    const timer = setInterval(() => sendVelocity(true), 100);
    return () => clearInterval(timer);
  }, [deadmanHeld, sendVelocity]);

  useEffect(() => {
    const subscription = AppState.addEventListener('change', (next) => {
      if (next !== 'active') releaseMotion();
    });
    return () => subscription.remove();
  }, [releaseMotion]);

  useEffect(() => {
    releaseMotion();
  }, [layout.orientation, releaseMotion]);

  useEffect(() => () => {
    inputRef.current = releaseHandheldInput(inputRef.current);
    if (bridgeReadyRef.current) sendVelocity(false);
  }, [sendVelocity]);

  useEffect(() => {
    if (!controlAvailable) releaseMotion();
  }, [controlAvailable, releaseMotion]);

  const cameraHtml = useMemo(() => robot
    ? `<!doctype html><meta name="viewport" content="width=device-width"><style>*{box-sizing:border-box}html,body{margin:0;background:#020504;height:100%;overflow:hidden}img{width:100%;height:100%;object-fit:cover}</style><img alt="RoboDog live camera" src="${buildEndpointUrl(robot, 'cameraMjpeg')}">`
    : '', [robot]);

  if (!robot) return <SafeAreaView style={styles.loading}>
    <Pressable style={styles.loadingBack} onPress={() => router.back()}>
      <Text style={styles.loadingBackText}>Back to robots</Text>
    </Pressable>
    {error
      ? <Text style={styles.error}>{error}</Text>
      : <><ActivityIndicator color={colors.accent} /><Text style={styles.muted}>Loading robot identity…</Text></>}
  </SafeAreaView>;

  const remoteUrl = buildRemoteControlUrl(robot);
  const bridgeUrl = bridgeUrlFor(remoteUrl);

  const handleBridgeMessage = (raw: string) => {
    try {
      const feedback = parseBridgeFeedback(raw);
      if (feedback.event === 'ready') {
        const nextLimits = Object.freeze({
          linear: numberField(feedback.limits, 'max_linear_velocity', DEFAULT_LIMITS.linear),
          angular: numberField(feedback.limits, 'max_angular_velocity', DEFAULT_LIMITS.angular),
        });
        limitsRef.current = nextLimits;
        bridgeReadyRef.current = true;
        setBridgeReady(true);
        setBridgeStatus('Control bridge ready');
      } else if (feedback.event === 'connection') {
        setBridgeConnected(feedback.connected);
        setBridgeStatus(feedback.message || (feedback.connected ? 'Connected' : 'Disconnected'));
        if (!feedback.connected) releaseMotion();
      } else if (feedback.event === 'state') {
        try { setState(parseRobotState(feedback.state)); } catch { /* HTTP telemetry remains available */ }
      } else if (feedback.event === 'ack') {
        setBridgeStatus(feedback.ok ? `${feedback.action ?? 'command'} accepted` : feedback.message);
      } else if (feedback.event === 'error') {
        setBridgeStatus(feedback.message);
      }
    } catch (reason) {
      setBridgeStatus(reason instanceof Error ? reason.message : 'Invalid control feedback');
    }
  };

  const sendGait = (gait: 'stand' | 'trot') => {
    if (!controlAvailable) return;
    releaseMotion();
    postEnvelope(buildGaitEnvelope(nextCommandId('gait'), gait));
  };
  const sendGreeting = () => {
    if (!controlAvailable) return;
    releaseMotion();
    postEnvelope(buildGreetingEnvelope(nextCommandId('greeting')));
  };
  const sendEstop = () => {
    if (!bridgeConnected) return;
    releaseMotion();
    postEnvelope(buildEstopEnvelope(nextCommandId('estop')));
  };

  if (view === 'web') return <SafeAreaView style={styles.webPage}>
    <View style={styles.webBar}>
      <View style={styles.webIdentity}>
        <Text style={styles.webTitle}>Robot Web Remote</Text>
        <Text numberOfLines={1} style={styles.webOrigin}>{robot.origin}</Text>
      </View>
      <Pressable style={styles.outlineButton} onPress={() => setView('cockpit')}>
        <Text style={styles.outlineText}>Cockpit</Text>
      </Pressable>
    </View>
    <WebView
      source={{ uri: remoteUrl }}
      style={styles.webview}
      javaScriptEnabled
      setSupportMultipleWindows={false}
      onShouldStartLoadWithRequest={(request) => isAllowedRobotNavigation(robot.origin, request.url)}
      onError={(event) => setError(event.nativeEvent.description)}
    />
  </SafeAreaView>;

  const metrics = <ScrollView
    horizontal
    showsHorizontalScrollIndicator={false}
    contentContainerStyle={styles.metricRail}
  >
    <Metric label="MODE" value={state?.mode ?? '—'} tone={colors.accent} />
    <Metric label="GAIT" value={state?.gait || '—'} tone={colors.cyan} />
    <Metric label="SPEED" value={`${(state?.velocity.vx ?? 0).toFixed(2)} m/s`} />
    <Metric label="SUPPLY" value={state?.power ? `${state.power.voltageV.toFixed(1)} V` : '—'} />
    <Metric
      label="HOTTEST"
      value={state?.telemetry
        ? `${state.telemetry.maxTemperatureC.toFixed(0)}° · ${state.telemetry.hottestJoint ?? 'joint'}`
        : '—'}
      tone={(state?.telemetry?.maxTemperatureC ?? 0) > 70 ? colors.danger : colors.text}
    />
    <Metric label="RMS" value={state?.telemetry ? `${state.telemetry.rmsTorqueNm.toFixed(1)} Nm` : '—'} />
    <Metric label="PEAK" value={state?.telemetry ? `${state.telemetry.peakTorqueNm.toFixed(1)} Nm` : '—'} />
  </ScrollView>;

  const visual = <View style={[styles.visualPane, layout.splitColumns && styles.visualPaneSplit]}>
    <View style={[styles.camera, { height: layout.cameraHeight }]}>
      <WebView
        source={{ html: cameraHtml, baseUrl: robot.origin }}
        scrollEnabled={false}
        style={styles.cameraWeb}
        onShouldStartLoadWithRequest={(request) =>
          request.url === 'about:blank' || isAllowedRobotNavigation(robot.origin, request.url)}
      />
      <View style={styles.cameraBadge}><Text style={styles.cameraBadgeText}>LIVE CAMERA</Text></View>
    </View>
    {metrics}
    <View style={styles.safetyLine}>
      <View style={[styles.statusDot, state?.safety.estop && styles.statusDanger]} />
      <Text numberOfLines={1} style={styles.safetyText}>
        {state?.safety.estop ? 'E-STOP ENGAGED' : error || 'Telemetry live'} · {state?.backend ?? 'connecting'}
      </Text>
    </View>
  </View>;

  const controls = <View style={[styles.controlsCard, layout.splitColumns && styles.controlsSplit]}>
    <View style={styles.controlHeader}>
      <View>
        <Text style={styles.cardTitle}>Handheld control</Text>
        <Text numberOfLines={1} style={styles.bridgeStatus}>{bridgeStatus}</Text>
      </View>
      <View style={[styles.readyPill, controlAvailable && styles.readyPillOn]}>
        <Text style={[styles.readyText, controlAvailable && styles.readyTextOn]}>
          {controlAvailable ? 'READY' : 'WAIT'}
        </Text>
      </View>
    </View>

    <View style={[styles.joysticks, { gap: layout.joystickGap }]}>
      <Joystick label="MOVE" size={layout.joystickDiameter} disabled={!controlAvailable}
        value={moveVisual} pressed={movePressed} gesture={joystickGestures.move} />
      <Joystick label="TURN" size={layout.joystickDiameter} disabled={!controlAvailable}
        value={turnVisual} pressed={turnPressed} gesture={joystickGestures.turn} />
    </View>

    <View style={[
      styles.deadman,
      deadmanHeld && styles.deadmanHeld,
      !controlAvailable && styles.controlDisabled,
    ]}>
      <Text style={[styles.deadmanText, deadmanHeld && styles.deadmanTextHeld]}>
        {deadmanHeld ? 'DRIVING · RELEASE STICKS TO STOP' : 'TOUCH A STICK TO DRIVE · DEADMAN'}
      </Text>
    </View>

    <View style={styles.actionRow}>
      <Pressable disabled={!controlAvailable} onPress={() => sendGait('stand')}
        style={[styles.action, !controlAvailable && styles.controlDisabled]}>
        <Text style={styles.actionText}>Stand</Text>
      </Pressable>
      <Pressable disabled={!controlAvailable} onPress={() => sendGait('trot')}
        style={[styles.action, styles.actionPrimary, !controlAvailable && styles.controlDisabled]}>
        <Text style={[styles.actionText, styles.actionPrimaryText]}>Walk</Text>
      </Pressable>
      <Pressable disabled={!controlAvailable} onPress={sendGreeting}
        style={[styles.action, !controlAvailable && styles.controlDisabled]}>
        <Text style={styles.actionText}>Greeting</Text>
      </Pressable>
    </View>

    <View style={styles.criticalRow}>
      <Pressable disabled={!bridgeConnected} onPress={sendEstop}
        style={[styles.estop, !bridgeConnected && styles.controlDisabled]}>
        <Text style={styles.estopText}>EMERGENCY STOP</Text>
      </Pressable>
      <Pressable onPress={() => { releaseMotion(); setView('web'); }} style={styles.webControl}>
        <Text style={styles.webControlText}>Web Remote</Text>
      </Pressable>
    </View>
  </View>;

  const gamepad = layout.landscapeGamepad;
  const landscapeGamepad = gamepad ? <View style={[styles.gamepadStage, { gap: gamepad.gap }]}>
    <View style={[styles.gamepadSidePane, { width: gamepad.sidePaneWidth }]}>
      <View style={[styles.gamepadSideActions, { height: gamepad.actionButtonHeight }]}>
        <Pressable hitSlop={LANDSCAPE_HIT_SLOP} disabled={!controlAvailable}
          onPress={() => sendGait('stand')}
          style={[styles.action, styles.gamepadSideButton, !controlAvailable && styles.controlDisabled]}>
          <Text numberOfLines={1} style={styles.gamepadActionText}>Stand</Text>
        </Pressable>
        <Pressable hitSlop={LANDSCAPE_HIT_SLOP} disabled={!controlAvailable}
          onPress={() => sendGait('trot')}
          style={[styles.action, styles.actionPrimary, styles.gamepadSideButton,
            !controlAvailable && styles.controlDisabled]}>
          <Text numberOfLines={1} style={[styles.gamepadActionText, styles.actionPrimaryText]}>Walk</Text>
        </Pressable>
      </View>
      <View style={styles.gamepadStickDock}>
        <Joystick label="MOVE" size={layout.joystickDiameter} disabled={!controlAvailable}
          value={moveVisual} pressed={movePressed} gesture={joystickGestures.move} />
      </View>
      <View style={[
        styles.deadman,
        styles.gamepadDeadman,
        deadmanHeld && styles.deadmanHeld,
        !controlAvailable && styles.controlDisabled,
      ]}>
        <Text numberOfLines={1} style={[
          styles.deadmanText,
          styles.gamepadDeadmanText,
          deadmanHeld && styles.deadmanTextHeld,
        ]}>
          {deadmanHeld ? 'STOP ON RELEASE' : 'DEADMAN READY'}
        </Text>
      </View>
    </View>

    <View style={[styles.gamepadCenter, { width: gamepad.centerPaneWidth }]}>
      <View style={[styles.camera, styles.gamepadCamera, { height: layout.cameraHeight }]}>
        <WebView
          source={{ html: cameraHtml, baseUrl: robot.origin }}
          scrollEnabled={false}
          style={styles.cameraWeb}
          onShouldStartLoadWithRequest={(request) =>
            request.url === 'about:blank' || isAllowedRobotNavigation(robot.origin, request.url)}
        />
        <View style={styles.cameraBadge}><Text style={styles.cameraBadgeText}>LIVE CAMERA</Text></View>
        <View style={styles.gamepadCameraStatus}>
          <View style={[styles.statusDot, state?.safety.estop && styles.statusDanger]} />
          <Text numberOfLines={1} style={styles.gamepadCameraStatusText}>
            {state?.safety.estop ? 'E-STOP ENGAGED' : error || bridgeStatus}
          </Text>
        </View>
      </View>
      {metrics}
    </View>

    <View style={[styles.gamepadSidePane, { width: gamepad.sidePaneWidth }]}>
      <View style={[styles.gamepadSideActions, { height: gamepad.actionButtonHeight }]}>
        <Pressable hitSlop={LANDSCAPE_HIT_SLOP} disabled={!controlAvailable} onPress={sendGreeting}
          style={[styles.action, styles.gamepadSideButton, !controlAvailable && styles.controlDisabled]}>
          <Text numberOfLines={1} style={styles.gamepadActionText}>Greet</Text>
        </Pressable>
        <Pressable hitSlop={LANDSCAPE_HIT_SLOP} disabled={!bridgeConnected} onPress={sendEstop}
          style={[styles.estop, styles.gamepadSideButton, !bridgeConnected && styles.controlDisabled]}>
          <Text numberOfLines={1} style={styles.estopText}>E-STOP</Text>
        </Pressable>
      </View>
      <View style={styles.gamepadStickDock}>
        <Joystick label="TURN" size={layout.joystickDiameter} disabled={!controlAvailable}
          value={turnVisual} pressed={turnPressed} gesture={joystickGestures.turn} />
      </View>
      <Pressable hitSlop={LANDSCAPE_HIT_SLOP}
        onPress={() => { releaseMotion(); setView('web'); }}
        style={[styles.webControl, styles.gamepadWebButton, { height: gamepad.actionButtonHeight }]}>
        <Text numberOfLines={1} style={styles.gamepadWebText}>Web</Text>
      </Pressable>
    </View>
  </View> : null;
  return <SafeAreaView style={styles.safe}>
    <View style={[
      styles.topline,
      gamepad && styles.gamepadTopline,
      { paddingHorizontal: layout.pagePadding },
    ]}>
      <Pressable
        accessibilityLabel="Back to robot selection"
        style={styles.backButton}
        onPress={() => { releaseMotion(); router.back(); }}
      >
        <Text style={styles.backButtonText}>‹</Text>
      </Pressable>
      <View style={styles.identity}>
        <Text numberOfLines={1} style={styles.robotName}>{robot.name}</Text>
        <Text numberOfLines={1} style={styles.origin}>{robot.origin}</Text>
      </View>
      {gamepad
        ? <View style={[styles.readyPill, controlAvailable && styles.readyPillOn]}>
            <Text style={[styles.readyText, controlAvailable && styles.readyTextOn]}>
              {controlAvailable ? 'READY' : 'WAIT'}
            </Text>
          </View>
        : <View style={styles.liveBadge}>
            <View style={styles.liveDot} /><Text style={styles.liveText}>LIVE</Text>
          </View>}
    </View>

    <View style={[
      styles.cockpit,
      { paddingHorizontal: layout.pagePadding },
      layout.splitColumns && !gamepad && styles.cockpitSplit,
      gamepad && styles.gamepadCockpit,
    ]}>
      {gamepad ? landscapeGamepad : <>
        {visual}
        {layout.splitColumns
          ? <ScrollView
            style={styles.controlsScroll}
            contentContainerStyle={styles.controlsScrollContent}
            showsVerticalScrollIndicator={false}
            nestedScrollEnabled
          >
            {controls}
          </ScrollView>
          : controls}
      </>}
    </View>

    <WebView
      ref={bridgeRef}
      source={{ uri: bridgeUrl }}
      style={styles.hiddenBridge}
      javaScriptEnabled
      setSupportMultipleWindows={false}
      onMessage={(event) => handleBridgeMessage(event.nativeEvent.data)}
      onShouldStartLoadWithRequest={(request) => isAllowedRobotNavigation(robot.origin, request.url)}
      onError={(event) => {
        bridgeReadyRef.current = false;
        setBridgeReady(false);
        setBridgeConnected(false);
        setBridgeStatus(event.nativeEvent.description);
        releaseMotion();
      }}
    />
  </SafeAreaView>;
}

const styles = StyleSheet.create({
  safe: { flex: 1, backgroundColor: colors.background },
  loading: { flex: 1, backgroundColor: colors.background, alignItems: 'center',
    justifyContent: 'center', gap: 12, padding: 24 },
  muted: { color: colors.muted },
  error: { color: colors.danger, textAlign: 'center' },
  loadingBack: { position: 'absolute', left: 18, top: 18, minHeight: 44,
    justifyContent: 'center', paddingHorizontal: 10 },
  loadingBackText: { color: colors.accent, fontWeight: '800' },
  topline: { minHeight: 50, paddingVertical: 7, flexDirection: 'row', gap: 12,
    justifyContent: 'space-between', alignItems: 'center' },
  gamepadTopline: { minHeight: 42, paddingVertical: 2 },
  identity: { flex: 1, minWidth: 0 },
  backButton: { width: 38, height: 38, borderRadius: 11, backgroundColor: colors.elevated,
    alignItems: 'center', justifyContent: 'center' },
  backButtonText: { color: colors.accent, fontSize: 30, lineHeight: 32, fontWeight: '700' },
  robotName: { color: colors.text, fontSize: 19, fontWeight: '900' },
  origin: { color: colors.muted, fontSize: 11, marginTop: 1 },
  liveBadge: { borderColor: colors.line, borderWidth: 1, paddingHorizontal: 9,
    paddingVertical: 5, borderRadius: 20, flexDirection: 'row', alignItems: 'center', gap: 5 },
  liveDot: { backgroundColor: colors.accent, width: 7, height: 7, borderRadius: 7 },
  liveText: { color: colors.accent, fontSize: 10, fontWeight: '900', letterSpacing: 1 },
  cockpit: { flex: 1, paddingBottom: 8, gap: 8 },
  cockpitSplit: { flexDirection: 'row', alignItems: 'stretch' },
  gamepadCockpit: { gap: 4, paddingBottom: 4 },
  gamepadStage: { flex: 1, minHeight: 0, flexDirection: 'row', alignItems: 'stretch' },
  gamepadSidePane: { flexShrink: 0, gap: 5, alignItems: 'stretch', justifyContent: 'space-between' },
  gamepadSideActions: { flexShrink: 0, flexDirection: 'row', gap: 4 },
  gamepadSideButton: { minHeight: 0, height: '100%', borderRadius: 8 },
  gamepadStickDock: { flex: 1, minHeight: 0, alignItems: 'center', justifyContent: 'center' },
  gamepadCenter: { flexShrink: 0, minWidth: 0, gap: 4, justifyContent: 'center' },
  controlsScroll: { flex: 1, minHeight: 0 },
  controlsScrollContent: { flexGrow: 1 },
  visualPane: { flex: 1, minHeight: 0, gap: 6 },
  visualPaneSplit: { flex: 1.15 },
  camera: { flexShrink: 0, backgroundColor: colors.black, borderRadius: 14,
    overflow: 'hidden', borderColor: colors.line, borderWidth: 1 },
  gamepadCamera: { borderRadius: 12 },
  cameraWeb: { flex: 1, backgroundColor: colors.black },
  cameraBadge: { position: 'absolute', left: 8, top: 8, backgroundColor: '#07110FCC',
    borderRadius: 6, paddingHorizontal: 7, paddingVertical: 4 },
  cameraBadgeText: { color: colors.cyan, fontSize: 8, fontWeight: '900', letterSpacing: 1 },
  gamepadCameraStatus: { position: 'absolute', left: 7, right: 7, bottom: 6,
    minHeight: 20, paddingHorizontal: 7, borderRadius: 6, backgroundColor: '#07110FCC',
    flexDirection: 'row', alignItems: 'center', gap: 6 },
  gamepadCameraStatusText: { flex: 1, color: colors.text, fontSize: 9, fontWeight: '700' },
  metricRail: { gap: 6, paddingVertical: 1 },
  metric: { backgroundColor: colors.elevated, borderRadius: 10, paddingHorizontal: 9,
    paddingVertical: 7, minWidth: 82, maxWidth: 128 },
  metricLabel: { color: colors.muted, fontSize: 8, letterSpacing: 1, fontWeight: '800' },
  metricValue: { marginTop: 2, fontWeight: '900', fontSize: 13 },
  safetyLine: { minHeight: 22, flexDirection: 'row', alignItems: 'center', gap: 7 },
  statusDot: { width: 7, height: 7, borderRadius: 7, backgroundColor: colors.accent },
  statusDanger: { backgroundColor: colors.danger },
  safetyText: { color: colors.muted, fontSize: 11, flex: 1 },
  controlsCard: { flexShrink: 0, backgroundColor: colors.panel, borderRadius: 16,
    padding: 9, borderWidth: 1, borderColor: colors.line, gap: 7 },
  controlsSplit: { flex: 1, alignSelf: 'stretch', justifyContent: 'space-between' },
  controlHeader: { minHeight: 32, flexDirection: 'row', alignItems: 'center',
    justifyContent: 'space-between', gap: 8 },
  cardTitle: { color: colors.text, fontSize: 16, fontWeight: '900' },
  bridgeStatus: { color: colors.muted, fontSize: 9, marginTop: 1, maxWidth: 220 },
  readyPill: { borderRadius: 9, backgroundColor: '#382D17', paddingHorizontal: 8, paddingVertical: 4 },
  readyPillOn: { backgroundColor: '#153D2E' },
  readyText: { color: colors.warning, fontSize: 9, fontWeight: '900', letterSpacing: 1 },
  readyTextOn: { color: colors.accent },
  joysticks: { flexDirection: 'row', justifyContent: 'space-around', alignItems: 'center' },
  deadman: { minHeight: 46, backgroundColor: colors.elevated, borderColor: colors.warning,
    borderWidth: 1, borderRadius: 11, alignItems: 'center', justifyContent: 'center' },
  gamepadDeadman: { minHeight: 26, height: 26, borderRadius: 8 },
  gamepadDeadmanText: { fontSize: 9 },
  deadmanHeld: { backgroundColor: colors.accent, borderColor: colors.accent },
  deadmanText: { color: colors.warning, fontWeight: '900', fontSize: 11, letterSpacing: 0.5 },
  deadmanTextHeld: { color: colors.black },
  controlDisabled: { opacity: 0.42 },
  actionRow: { flexDirection: 'row', gap: 6 },
  action: { minHeight: 44, flex: 1, backgroundColor: colors.elevated, borderRadius: 10,
    alignItems: 'center', justifyContent: 'center' },
  gamepadActionText: { color: colors.text, fontWeight: '800', fontSize: 10 },
  gamepadWebButton: { minHeight: 0, width: '100%', borderRadius: 8 },
  gamepadWebText: { color: colors.black, fontWeight: '900', fontSize: 9 },
  actionPrimary: { backgroundColor: colors.accent },
  actionText: { color: colors.text, fontWeight: '800', fontSize: 12 },
  actionPrimaryText: { color: colors.black },
  criticalRow: { flexDirection: 'row', gap: 6 },
  estop: { minHeight: 46, flex: 1.35, backgroundColor: '#48171B', borderRadius: 10,
    alignItems: 'center', justifyContent: 'center' },
  estopText: { color: colors.danger, fontWeight: '900', fontSize: 10, letterSpacing: 0.5 },
  webControl: { minHeight: 46, flex: 1, backgroundColor: colors.cyan, borderRadius: 10,
    alignItems: 'center', justifyContent: 'center' },
  webControlText: { color: colors.black, fontWeight: '900', fontSize: 12 },
  hiddenBridge: { position: 'absolute', width: 1, height: 1, opacity: 0, left: -10, bottom: 0 },
  webPage: { flex: 1, backgroundColor: colors.background },
  webBar: { minHeight: 56, padding: 10, paddingHorizontal: 14, flexDirection: 'row',
    justifyContent: 'space-between', alignItems: 'center', gap: 10,
    borderBottomColor: colors.line, borderBottomWidth: 1 },
  webIdentity: { minWidth: 0, flex: 1 },
  webTitle: { color: colors.text, fontWeight: '900' },
  webOrigin: { color: colors.muted, fontSize: 10, marginTop: 2 },
  outlineButton: { borderColor: colors.accent, borderWidth: 1, borderRadius: 10,
    paddingVertical: 8, paddingHorizontal: 13 },
  outlineText: { color: colors.accent, fontWeight: '800' },
  webview: { flex: 1, backgroundColor: colors.background },
});

import { useLocalSearchParams } from 'expo-router';
import { useEffect, useMemo, useState } from 'react';
import { ActivityIndicator, AppState, Pressable, SafeAreaView, ScrollView, StyleSheet, Text, useWindowDimensions, View } from 'react-native';
import { WebView } from 'react-native-webview';

import { Joystick } from '../components/Joystick';
import { buildEndpointUrl, buildRemoteControlUrl, isAllowedRobotNavigation } from '../domain/robot';
import type { RobotDescriptor, RobotState } from '../domain/robot';
import { connectManualRobot, fetchRobotState } from '../services/robotClient';
import { colors } from '../theme';

type ViewMode = 'cockpit' | 'web';

const metric = (label: string, value: string, tone: string = colors.text) => <View style={styles.metric}><Text style={styles.metricLabel}>{label}</Text><Text style={[styles.metricValue, { color: tone }]}>{value}</Text></View>;

export default function ControlScreen() {
  const params = useLocalSearchParams<{ origin?: string }>();
  const { width, height } = useWindowDimensions();
  const landscape = width > height && width >= 720;
  const [robot, setRobot] = useState<RobotDescriptor | null>(null);
  const [state, setState] = useState<RobotState | null>(null);
  const [error, setError] = useState('');
  const [view, setView] = useState<ViewMode>('cockpit');

  useEffect(() => {
    let active = true;
    if (!params.origin) { setError('Missing robot address'); return; }
    connectManualRobot(params.origin).then((value) => { if (active) setRobot(value); }).catch((reason) => { if (active) setError(reason instanceof Error ? reason.message : 'Connection failed'); });
    return () => { active = false; };
  }, [params.origin]);

  useEffect(() => {
    if (!robot) return;
    let active = true;
    const refresh = async () => {
      if (AppState.currentState !== 'active') return;
      try { const next = await fetchRobotState(robot); if (active) { setState(next); setError(''); } }
      catch (reason) { if (active) setError(reason instanceof Error ? reason.message : 'Telemetry unavailable'); }
    };
    void refresh();
    const timer = setInterval(refresh, 500);
    return () => { active = false; clearInterval(timer); };
  }, [robot]);

  const cameraHtml = useMemo(() => robot ? `<!doctype html><meta name="viewport" content="width=device-width"><style>*{box-sizing:border-box}html,body{margin:0;background:#020504;height:100%;overflow:hidden}img{width:100%;height:100%;object-fit:cover}</style><img alt="RoboDog live camera" src="${buildEndpointUrl(robot, 'cameraMjpeg')}">` : '', [robot]);

  if (!robot) return <SafeAreaView style={styles.loading}>{error ? <Text style={styles.error}>{error}</Text> : <><ActivityIndicator color={colors.accent} /><Text style={styles.muted}>Loading robot identity…</Text></>}</SafeAreaView>;
  const remoteUrl = buildRemoteControlUrl(robot);
  if (view === 'web') return <SafeAreaView style={styles.webPage}>
    <View style={styles.webBar}><View><Text style={styles.webTitle}>Trusted-LAN web control</Text><Text style={styles.webOrigin}>{robot.origin}</Text></View><Pressable style={styles.outlineButton} onPress={() => setView('cockpit')}><Text style={styles.outlineText}>Cockpit</Text></Pressable></View>
    <WebView
      source={{ uri: remoteUrl }}
      style={styles.webview}
      javaScriptEnabled
      setSupportMultipleWindows={false}
      onShouldStartLoadWithRequest={(request) => isAllowedRobotNavigation(robot.origin, request.url)}
      onError={(event) => setError(event.nativeEvent.description)}
    />
  </SafeAreaView>;

  const telemetry = <View style={styles.telemetry}>
    <View style={styles.metricGrid}>
      {metric('MODE', state?.mode ?? '—', colors.accent)}
      {metric('GAIT', state?.gait || '—', colors.cyan)}
      {metric('SPEED', `${(state?.velocity.vx ?? 0).toFixed(2)} m/s`)}
      {metric('VOLTAGE', state?.power ? `${state.power.voltageV.toFixed(1)} V` : '—')}
      {metric('HOTTEST', state?.telemetry ? `${state.telemetry.maxTemperatureC.toFixed(0)} °C · ${state.telemetry.hottestJoint ?? 'joint'}` : '—', (state?.telemetry?.maxTemperatureC ?? 0) > 70 ? colors.danger : colors.text)}
      {metric('TORQUE RMS', state?.telemetry ? `${state.telemetry.rmsTorqueNm.toFixed(1)} Nm` : '—')}
      {metric('TORQUE PEAK', state?.telemetry ? `${state.telemetry.peakTorqueNm.toFixed(1)} Nm` : '—')}
    </View>
    <View style={styles.safetyLine}><View style={[styles.statusDot, state?.safety.estop && styles.statusDanger]} /><Text style={styles.safetyText}>{state?.safety.estop ? 'E-STOP ENGAGED' : error || 'Telemetry live'} · {state?.backend ?? 'connecting'}</Text></View>
  </View>;

  const controls = <View style={styles.controlsCard}>
    <View style={styles.lockRow}><Text style={styles.cardTitle}>Native controls</Text><Text style={styles.lockPill}>LOCKED</Text></View>
    <Text style={styles.lockCopy}>Use trusted-LAN web control today. Native motion unlocks after pairing, TLS and the scoped command endpoint are available.</Text>
    <View style={styles.joysticks}><Joystick label="MOVE" disabled /><Joystick label="TURN" disabled /></View>
    <Pressable disabled style={styles.deadman}><Text style={styles.deadmanText}>HOLD TO MOVE · DEADMAN</Text></Pressable>
    <View style={styles.actionRow}>{['Stand', 'Walk', 'Greeting'].map((label) => <Pressable disabled key={label} style={styles.action}><Text style={styles.actionText}>{label}</Text></Pressable>)}</View>
    <Pressable disabled style={styles.estop}><Text style={styles.estopText}>EMERGENCY STOP</Text></Pressable>
    <Pressable style={styles.webControl} onPress={() => setView('web')}><Text style={styles.webControlText}>Open working web remote</Text><Text style={styles.webControlSub}>Same-origin control · camera · greeting</Text></Pressable>
  </View>;

  return <SafeAreaView style={styles.safe}><ScrollView contentContainerStyle={[styles.page, landscape && styles.pageLandscape]}>
    <View style={styles.topline}><View><Text style={styles.robotName}>{robot.name}</Text><Text style={styles.origin}>{robot.origin}</Text></View><View style={styles.liveBadge}><View style={styles.liveDot} /><Text style={styles.liveText}>LIVE</Text></View></View>
    <View style={[styles.columns, landscape && styles.columnsLandscape]}>
      <View style={styles.visualColumn}><View style={styles.camera}><WebView source={{ html: cameraHtml, baseUrl: robot.origin }} scrollEnabled={false} style={styles.cameraWeb} onShouldStartLoadWithRequest={(request) => request.url === 'about:blank' || isAllowedRobotNavigation(robot.origin, request.url)} /><View style={styles.cameraBadge}><Text style={styles.cameraBadgeText}>MJPEG PREVIEW · CURRENT</Text></View></View>{telemetry}</View>
      <View style={styles.controlColumn}>{controls}</View>
    </View>
    <Text style={styles.mediaRoadmap}>Media transport boundary: MJPEG preview now. The planned 60 fps path replaces only this surface with WebRTC/H.264 hardware decode.</Text>
  </ScrollView></SafeAreaView>;
}

const styles = StyleSheet.create({
  safe: { flex: 1, backgroundColor: colors.background }, loading: { flex: 1, backgroundColor: colors.background, alignItems: 'center', justifyContent: 'center', gap: 12, padding: 24 }, muted: { color: colors.muted }, error: { color: colors.danger, textAlign: 'center' }, page: { padding: 16, gap: 14, maxWidth: 1180, width: '100%', alignSelf: 'center' }, pageLandscape: { paddingHorizontal: 24 },
  topline: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' }, robotName: { color: colors.text, fontSize: 24, fontWeight: '900' }, origin: { color: colors.muted, fontSize: 12, marginTop: 2 }, liveBadge: { borderColor: colors.line, borderWidth: 1, paddingHorizontal: 10, paddingVertical: 6, borderRadius: 20, flexDirection: 'row', alignItems: 'center', gap: 6 }, liveDot: { backgroundColor: colors.accent, width: 7, height: 7, borderRadius: 7 }, liveText: { color: colors.accent, fontSize: 11, fontWeight: '900', letterSpacing: 1 },
  columns: { gap: 14 }, columnsLandscape: { flexDirection: 'row' }, visualColumn: { flex: 1.25, gap: 12 }, controlColumn: { flex: 1 }, camera: { aspectRatio: 16 / 9, backgroundColor: colors.black, borderRadius: 18, overflow: 'hidden', borderColor: colors.line, borderWidth: 1 }, cameraWeb: { backgroundColor: colors.black }, cameraBadge: { position: 'absolute', left: 10, top: 10, backgroundColor: '#07110FCC', borderRadius: 7, paddingHorizontal: 8, paddingVertical: 5 }, cameraBadgeText: { color: colors.cyan, fontSize: 9, fontWeight: '800', letterSpacing: 1 },
  telemetry: { backgroundColor: colors.panel, borderRadius: 18, padding: 14, borderWidth: 1, borderColor: colors.line, gap: 12 }, metricGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 }, metric: { backgroundColor: colors.elevated, borderRadius: 12, padding: 10, minWidth: 94, flexGrow: 1 }, metricLabel: { color: colors.muted, fontSize: 9, letterSpacing: 1, fontWeight: '800' }, metricValue: { marginTop: 4, fontWeight: '900', fontSize: 16 }, safetyLine: { flexDirection: 'row', alignItems: 'center', gap: 8 }, statusDot: { width: 8, height: 8, borderRadius: 8, backgroundColor: colors.accent }, statusDanger: { backgroundColor: colors.danger }, safetyText: { color: colors.muted, fontSize: 12, flex: 1 },
  controlsCard: { backgroundColor: colors.panel, borderRadius: 18, padding: 15, borderWidth: 1, borderColor: colors.line, gap: 12 }, lockRow: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' }, cardTitle: { color: colors.text, fontSize: 19, fontWeight: '900' }, lockPill: { color: colors.warning, backgroundColor: '#3A2B10', paddingHorizontal: 9, paddingVertical: 5, borderRadius: 10, fontSize: 10, fontWeight: '900' }, lockCopy: { color: colors.muted, fontSize: 12, lineHeight: 18 }, joysticks: { flexDirection: 'row', justifyContent: 'space-around', gap: 10, flexWrap: 'wrap' }, deadman: { backgroundColor: colors.elevated, borderColor: colors.warning, borderWidth: 1, borderRadius: 12, padding: 13, alignItems: 'center', opacity: 0.65 }, deadmanText: { color: colors.warning, fontWeight: '900', fontSize: 12 }, actionRow: { flexDirection: 'row', gap: 8 }, action: { flex: 1, backgroundColor: colors.elevated, padding: 12, borderRadius: 11, alignItems: 'center', opacity: 0.65 }, actionText: { color: colors.text, fontWeight: '800' }, estop: { backgroundColor: '#48171B', padding: 15, borderRadius: 12, alignItems: 'center', opacity: 0.7 }, estopText: { color: colors.danger, fontWeight: '900', letterSpacing: 1 }, webControl: { backgroundColor: colors.accent, padding: 14, borderRadius: 13, alignItems: 'center' }, webControlText: { color: colors.black, fontWeight: '900', fontSize: 16 }, webControlSub: { color: '#18402F', fontSize: 11, marginTop: 2 }, mediaRoadmap: { color: colors.muted, fontSize: 11, lineHeight: 17, marginBottom: 12 },
  webPage: { flex: 1, backgroundColor: colors.background }, webBar: { padding: 12, paddingHorizontal: 16, flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', borderBottomColor: colors.line, borderBottomWidth: 1 }, webTitle: { color: colors.text, fontWeight: '900' }, webOrigin: { color: colors.muted, fontSize: 11 }, outlineButton: { borderColor: colors.accent, borderWidth: 1, borderRadius: 10, paddingVertical: 8, paddingHorizontal: 13 }, outlineText: { color: colors.accent, fontWeight: '800' }, webview: { flex: 1, backgroundColor: colors.background },
});

import { router } from 'expo-router';
import { useEffect, useMemo, useState } from 'react';
import { ActivityIndicator, Pressable, ScrollView, StyleSheet, Text, TextInput, View } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';

import type { RobotDescriptor } from '../domain/robot';
import { mergeDiscoveredRobots, parseMdnsService } from '../services/discovery';
import { nativeDiscoveryAvailable, nativeRobotDiscovery } from '../services/nativeDiscovery';
import { connectDiscoveredRobot, connectManualRobot } from '../services/robotClient';
import { colors } from '../theme';

export default function DiscoveryScreen() {
  const [robots, setRobots] = useState<readonly RobotDescriptor[]>([]);
  const [manualHost, setManualHost] = useState('');
  const [status, setStatus] = useState(nativeDiscoveryAvailable ? 'Searching the local network…' : 'Native discovery needs a development build.');
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    let active = true;
    let cleanup: () => void = () => undefined;
    nativeRobotDiscovery.start(async (raw) => {
      if (!active || !parseMdnsService(raw)) return;
      try {
        const robot = await connectDiscoveredRobot(raw);
        if (active) {
          setRobots((current) => mergeDiscoveredRobots(current, [robot]));
          setStatus('Robot found');
        }
      } catch (error) {
        if (active) setStatus(error instanceof Error ? error.message : 'Discovery failed');
      }
    }).then((stop) => { cleanup = stop; });
    return () => { active = false; cleanup(); };
  }, []);

  const openRobot = (robot: RobotDescriptor) => router.push({ pathname: '/control', params: { origin: robot.origin } });
  const connectManual = async () => {
    setBusy(true);
    try {
      const robot = await connectManualRobot(manualHost);
      setRobots((current) => mergeDiscoveredRobots(current, [robot]));
      openRobot(robot);
    } catch (error) {
      setStatus(error instanceof Error ? error.message : 'Could not connect');
    } finally { setBusy(false); }
  };
  const cards = useMemo(() => robots.map((robot) => (
    <Pressable key={robot.id} style={styles.robotCard} onPress={() => openRobot(robot)} accessibilityRole="button">
      <View style={styles.robotGlyph}><Text style={styles.robotGlyphText}>R</Text></View>
      <View style={styles.robotCopy}><Text style={styles.robotName}>{robot.name}</Text><Text style={styles.robotMeta}>{robot.model} · API {robot.apiVersion}</Text><Text style={styles.robotOrigin}>{robot.origin}</Text></View>
      <Text style={styles.chevron}>›</Text>
    </Pressable>
  )), [robots]);

  return <SafeAreaView style={styles.safe}><ScrollView contentContainerStyle={styles.page} keyboardShouldPersistTaps="handled">
    <View style={styles.brand}><View style={styles.logo}><Text style={styles.logoText}>RD</Text></View><View><Text style={styles.eyebrow}>FIELD CONTROL</Text><Text style={styles.title}>RoboDog</Text></View></View>
    <View style={styles.hero}><Text style={styles.heroTitle}>Ready when the robot is.</Text><Text style={styles.heroBody}>Connect on the same trusted local network. Motion stays inside the robot’s same-origin web controller.</Text></View>
    <View style={styles.statusRow}><View style={[styles.dot, nativeDiscoveryAvailable && styles.dotLive]} />{nativeDiscoveryAvailable && <ActivityIndicator color={colors.accent} size="small" />}<Text style={styles.statusText}>{status}</Text></View>
    {cards}
    <View style={styles.manualCard}><Text style={styles.sectionTitle}>Connect manually</Text><Text style={styles.hint}>Use the robot hostname or LAN IP, for example 192.168.1.42:8080.</Text>
      <TextInput value={manualHost} onChangeText={setManualHost} autoCapitalize="none" autoCorrect={false} keyboardType="url" placeholder="robodog.local:8080" placeholderTextColor={colors.muted} style={styles.input} />
      <Pressable disabled={busy || !manualHost.trim()} onPress={connectManual} style={({ pressed }) => [styles.primary, (pressed || busy) && styles.pressed]}><Text style={styles.primaryText}>{busy ? 'Connecting…' : 'Connect on LAN'}</Text></Pressable>
    </View>
    <Text style={styles.footer}>Discovery broadcasts identity only. Direct native motion remains locked until the authenticated native command protocol is available.</Text>
  </ScrollView></SafeAreaView>;
}

const styles = StyleSheet.create({
  safe: { flex: 1, backgroundColor: colors.background }, page: { padding: 22, gap: 16, maxWidth: 760, width: '100%', alignSelf: 'center' },
  brand: { flexDirection: 'row', alignItems: 'center', gap: 12, marginTop: 18 }, logo: { width: 48, height: 48, borderRadius: 15, backgroundColor: colors.accent, alignItems: 'center', justifyContent: 'center' }, logoText: { color: colors.black, fontWeight: '900' }, eyebrow: { color: colors.accent, fontSize: 11, letterSpacing: 2, fontWeight: '800' }, title: { color: colors.text, fontSize: 26, fontWeight: '800' },
  hero: { marginVertical: 20 }, heroTitle: { color: colors.text, fontSize: 34, lineHeight: 39, fontWeight: '800' }, heroBody: { color: colors.muted, fontSize: 16, lineHeight: 23, marginTop: 8 },
  statusRow: { flexDirection: 'row', alignItems: 'center', gap: 8 }, dot: { width: 9, height: 9, borderRadius: 9, backgroundColor: colors.warning }, dotLive: { backgroundColor: colors.accent }, statusText: { color: colors.muted, flex: 1 },
  robotCard: { backgroundColor: colors.panel, borderColor: colors.line, borderWidth: 1, borderRadius: 18, padding: 16, flexDirection: 'row', alignItems: 'center', gap: 13 }, robotGlyph: { width: 46, height: 46, borderRadius: 14, backgroundColor: colors.elevated, alignItems: 'center', justifyContent: 'center' }, robotGlyphText: { color: colors.accent, fontWeight: '900', fontSize: 20 }, robotCopy: { flex: 1 }, robotName: { color: colors.text, fontWeight: '800', fontSize: 17 }, robotMeta: { color: colors.muted, marginTop: 2 }, robotOrigin: { color: colors.cyan, marginTop: 4, fontSize: 12 }, chevron: { color: colors.accent, fontSize: 30 },
  manualCard: { backgroundColor: colors.panel, padding: 18, borderRadius: 20, gap: 11, marginTop: 8 }, sectionTitle: { color: colors.text, fontSize: 19, fontWeight: '800' }, hint: { color: colors.muted, lineHeight: 20 }, input: { backgroundColor: colors.background, borderColor: colors.line, borderWidth: 1, borderRadius: 13, color: colors.text, padding: 14, fontSize: 16 }, primary: { backgroundColor: colors.accent, borderRadius: 13, padding: 15, alignItems: 'center' }, primaryText: { color: colors.black, fontWeight: '900' }, pressed: { opacity: 0.6 }, footer: { color: colors.muted, fontSize: 12, lineHeight: 18, marginVertical: 16 },
});

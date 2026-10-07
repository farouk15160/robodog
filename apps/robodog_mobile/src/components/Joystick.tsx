import { useMemo, useRef, useState } from 'react';
import { PanResponder, StyleSheet, Text, View } from 'react-native';

import { clampJoystick } from '../domain/joystick';
import type { JoystickVector } from '../domain/joystick';
import { colors } from '../theme';

interface Props {
  readonly label: string;
  readonly disabled?: boolean;
  readonly onChange?: (value: JoystickVector) => void;
}

const RADIUS = 54;

export function Joystick({ label, disabled = false, onChange }: Props) {
  const [value, setValue] = useState<JoystickVector>({ x: 0, y: 0 });
  const emit = (next: JoystickVector) => { setValue(next); onChange?.(next); };
  const enabledRef = useRef(!disabled);
  enabledRef.current = !disabled;
  const responder = useMemo(() => PanResponder.create({
    onStartShouldSetPanResponder: () => enabledRef.current,
    onMoveShouldSetPanResponder: () => enabledRef.current,
    onPanResponderMove: (_, gesture) => emit(clampJoystick({ x: gesture.dx / RADIUS, y: gesture.dy / RADIUS })),
    onPanResponderRelease: () => emit({ x: 0, y: 0 }),
    onPanResponderTerminate: () => emit({ x: 0, y: 0 }),
  }), []);
  return <View style={styles.wrapper} accessibilityLabel={`${label} joystick${disabled ? ', locked' : ''}`}>
    <View style={[styles.track, disabled && styles.disabled]} {...responder.panHandlers}>
      <View style={[styles.knob, { transform: [{ translateX: value.x * RADIUS }, { translateY: value.y * RADIUS }] }]} />
      <View style={styles.crossHorizontal} /><View style={styles.crossVertical} />
    </View>
    <Text style={styles.label}>{label}</Text>
  </View>;
}

const styles = StyleSheet.create({
  wrapper: { alignItems: 'center', gap: 9 }, track: { width: 142, height: 142, borderRadius: 71, backgroundColor: colors.elevated, borderColor: colors.line, borderWidth: 1, alignItems: 'center', justifyContent: 'center', overflow: 'hidden' }, disabled: { opacity: 0.68 }, knob: { width: 52, height: 52, borderRadius: 26, backgroundColor: colors.accent, position: 'absolute', zIndex: 2 }, crossHorizontal: { width: 110, height: 1, backgroundColor: colors.line }, crossVertical: { height: 110, width: 1, backgroundColor: colors.line, position: 'absolute' }, label: { color: colors.muted, fontSize: 12, fontWeight: '700', letterSpacing: 1 },
});

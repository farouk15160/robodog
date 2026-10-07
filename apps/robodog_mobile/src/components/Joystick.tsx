import { StyleSheet, Text, View } from 'react-native';
import { GestureDetector } from 'react-native-gesture-handler';
import type { PanGesture } from 'react-native-gesture-handler';

import { joystickGeometry, joystickTranslation } from '../domain/joystick';
import type { JoystickVector } from '../domain/joystick';
import { colors } from '../theme';

interface Props {
  readonly label: string;
  readonly disabled?: boolean;
  readonly size?: number;
  readonly value: JoystickVector;
  readonly pressed: boolean;
  readonly gesture: PanGesture;
}

export function Joystick({
  label,
  disabled = false,
  size = 142,
  value,
  pressed,
  gesture,
}: Props) {
  const geometry = joystickGeometry(size);
  const translated = joystickTranslation(value, geometry.diameter);
  return <View
    style={styles.wrapper}
    accessibilityLabel={`${label} joystick${disabled ? ', unavailable' : ''}`}
    accessibilityState={{ disabled }}
  >
    <GestureDetector gesture={gesture}>
      <View
      style={[
        styles.track,
        { width: geometry.diameter, height: geometry.diameter,
          borderRadius: geometry.diameter / 2 },
        pressed && styles.pressed,
        disabled && styles.disabled,
      ]}
      >
        <View style={[
          styles.knob,
          { width: geometry.knobDiameter, height: geometry.knobDiameter,
            borderRadius: geometry.knobDiameter / 2,
            transform: [{ translateX: translated.x }, { translateY: translated.y }] },
        ]} />
        <View style={[styles.crossHorizontal, { width: geometry.diameter * 0.77 }]} />
        <View style={[styles.crossVertical, { height: geometry.diameter * 0.77 }]} />
      </View>
    </GestureDetector>
    <Text style={styles.label}>{label}</Text>
  </View>;
}

const styles = StyleSheet.create({
  wrapper: { alignItems: 'center', gap: 6 },
  track: { backgroundColor: colors.elevated, borderColor: colors.line, borderWidth: 2,
    alignItems: 'center', justifyContent: 'center', overflow: 'hidden' },
  pressed: { borderColor: colors.accent, backgroundColor: '#1C3C33' },
  disabled: { opacity: 0.5 },
  knob: { backgroundColor: colors.accent, position: 'absolute', zIndex: 2 },
  crossHorizontal: { height: 1, backgroundColor: colors.line },
  crossVertical: { width: 1, backgroundColor: colors.line, position: 'absolute' },
  label: { color: colors.muted, fontSize: 11, fontWeight: '800', letterSpacing: 1 },
});

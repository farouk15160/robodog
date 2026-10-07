import { useEffect, useMemo, useRef, useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';

import { clampJoystick, joystickGeometry, joystickTranslation } from '../domain/joystick';
import type { JoystickVector } from '../domain/joystick';
import { colors } from '../theme';

interface Props {
  readonly label: string;
  readonly disabled?: boolean;
  readonly size?: number;
  readonly resetKey?: number;
  readonly onChange?: (value: JoystickVector) => void;
  readonly onActiveChange?: (active: boolean) => void;
}

export function Joystick({
  label,
  disabled = false,
  size = 142,
  resetKey = 0,
  onChange,
  onActiveChange,
}: Props) {
  const [value, setValue] = useState<JoystickVector>({ x: 0, y: 0 });
  const [pressed, setPressed] = useState(false);
  const geometry = joystickGeometry(size);
  const translated = joystickTranslation(value, geometry.diameter);
  const onChangeRef = useRef(onChange);
  onChangeRef.current = onChange;
  const onActiveChangeRef = useRef(onActiveChange);
  onActiveChangeRef.current = onActiveChange;
  const emit = (next: JoystickVector) => {
    setValue(next);
    onChangeRef.current?.(next);
  };
  useEffect(() => {
    setPressed(false);
    setValue({ x: 0, y: 0 });
  }, [resetKey]);
  const gesture = useMemo(() => Gesture.Pan()
    .enabled(!disabled)
    .minDistance(0)
    .shouldCancelWhenOutside(false)
    .runOnJS(true)
    .onBegin(() => { setPressed(true); onActiveChangeRef.current?.(true); })
    .onUpdate((event) => emit(clampJoystick({
      x: event.translationX / geometry.travel,
      y: event.translationY / geometry.travel,
    })))
    .onFinalize(() => {
      setPressed(false); emit({ x: 0, y: 0 }); onActiveChangeRef.current?.(false);
    }), [disabled, geometry.travel]);
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

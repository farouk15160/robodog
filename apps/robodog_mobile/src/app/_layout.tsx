import { Stack } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { SafeAreaProvider } from 'react-native-safe-area-context';

import { colors } from '../theme';

export default function RootLayout() {
  return <GestureHandlerRootView style={{ flex: 1 }}>
    <SafeAreaProvider>
      <StatusBar style="light" />
      <Stack screenOptions={{ headerStyle: { backgroundColor: colors.background }, headerTintColor: colors.text, contentStyle: { backgroundColor: colors.background } }}>
        <Stack.Screen name="index" options={{ title: 'RoboDog Control', headerShown: false }} />
        <Stack.Screen name="control" options={{ title: 'Control cockpit', headerShown: false }} />
      </Stack>
    </SafeAreaProvider>
  </GestureHandlerRootView>;
}

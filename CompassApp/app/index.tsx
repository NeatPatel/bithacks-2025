import React, { useEffect, useState } from 'react';
import {
  View,
  Text,
  StyleSheet,
  PermissionsAndroid,
  Platform,
} from 'react-native';
import getLocation from '@/components/getLocation';

const App = () => {
  const [location, setLocation] = useState<{
    latitude: number;
    longitude: number;
  } | null>(null);

  useEffect(() => {
    let intervalId: NodeJS.Timeout;

    const requestPermission = async () => {
      if (Platform.OS === 'android') {
        const granted = await PermissionsAndroid.request(
          PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
          {
            title: 'Location Permission',
            message: 'This app needs access to your location.',
            buttonPositive: 'OK',
          }
        );
        return granted === PermissionsAndroid.RESULTS.GRANTED;
      }
      return true;
    };

    const startTracking = async () => {
      const hasPermission = await requestPermission();
      if (!hasPermission) {
        console.warn('Permission denied');
        return;
      }

      // Get location once immediately
      const initial = await getLocation();
      if (initial) {
        setLocation({ latitude: initial.latitude, longitude: initial.longitude });
      }

      // Then update every 5 seconds
      intervalId = setInterval(async () => {
        const updated = await getLocation();
        if (updated) {
          setLocation({
            latitude: updated.latitude,
            longitude: updated.longitude,
          });
        }
      }, 100);
    };

    startTracking();

    return () => {
      if (intervalId) clearInterval(intervalId);
    };
  }, []);

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Real-Time Location</Text>
      {location ? (
        <>
          <Text style={styles.text}>Latitude: {location.latitude}</Text>
          <Text style={styles.text}>Longitude: {location.longitude}</Text>
        </>
      ) : (
        <Text style={styles.text}>Getting location...</Text>
      )}
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 16,
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    marginBottom: 12,
  },
  text: {
    fontSize: 18,
    marginVertical: 4,
  },
});

export default App;

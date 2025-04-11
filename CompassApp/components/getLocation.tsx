import * as GetLocation from 'expo-location';

const getLocation = async () => {
  try {
    const { status } = await GetLocation.requestForegroundPermissionsAsync();
    const statusBackground = await GetLocation.requestBackgroundPermissionsAsync();
    const location = await GetLocation.getCurrentPositionAsync({
      accuracy: 6,
      timeInterval: 1000,
      distanceInterval: 0
    });

    return location.coords;
  } catch (error: any) {
    if (error && error.code && error.message) {
      console.warn(`Location error [${error.code}]: ${error.message}`);
    } else {
      console.warn('Unknown location error', error);
    }
    return null;
  }
};

export default getLocation;

import type { CapacitorConfig } from "@capacitor/cli";

// Native shells for iOS and Android. `npx cap add ios` / `npx cap add android`
// create the ios/ and android/ projects (not versioned until the first release).
const config: CapacitorConfig = {
  appId: "com.acwing.pilot",
  appName: "ACwing Pilot",
  webDir: "dist",
  backgroundColor: "#0A0B0D",
  plugins: {
    BluetoothLe: {
      displayStrings: {
        scanning: "Searching for ACwing drones…",
        cancel: "Cancel",
        availableDevices: "ACwing drones nearby",
        noDeviceFound: "No ACwing drone found",
      },
    },
  },
};

export default config;

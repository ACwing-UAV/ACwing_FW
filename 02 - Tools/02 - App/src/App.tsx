import { useEffect } from "react";
import { useRoute, useSettings } from "./ui/hooks";
import { Toasts } from "./ui/kit";
import { ConnectScreen } from "./screens/ConnectScreen";
import { CockpitScreen } from "./screens/CockpitScreen";
import { TelemetryScreen } from "./screens/TelemetryScreen";
import { SettingsScreen } from "./screens/settings/SettingsScreen";
import { HelpScreen } from "./screens/HelpScreen";
import { CalibRoute } from "./calib/wizards";

export function App() {
  const [page, arg] = useRoute();
  const st = useSettings();
  useEffect(() => {
    document.documentElement.dataset.theme = st.theme;
    document.documentElement.lang = st.lang;
    document.querySelector('meta[name="theme-color"]')?.setAttribute("content", st.theme === "day" ? "#F4F6F8" : "#0E1116");
  }, [st.theme, st.lang]);

  let screen;
  switch (page) {
    case "cockpit": screen = <CockpitScreen />; break;
    case "telemetry": screen = <TelemetryScreen />; break;
    case "settings": screen = <SettingsScreen section={arg} />; break;
    case "calib": screen = <CalibRoute key={arg} type={Number(arg)} />; break;
    case "help": screen = <HelpScreen />; break;
    default: screen = <ConnectScreen />;
  }
  return (
    <>
      {screen}
      <Toasts />
    </>
  );
}

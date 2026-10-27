import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { Capacitor } from "@capacitor/core";
import { App } from "./App";
import { flightLog } from "./link/flightLog";
import { startAlerts } from "./alerts/alerts";
import "./styles.css";

flightLog.start();
startAlerts();

// Offline cache for the installed web app (PWA). The native iOS/Android shells bundle the files already.
if (import.meta.env.PROD && !Capacitor.isNativePlatform() && "serviceWorker" in navigator) {
  addEventListener("load", () => navigator.serviceWorker.register("./sw.js").catch(() => undefined));
}

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);

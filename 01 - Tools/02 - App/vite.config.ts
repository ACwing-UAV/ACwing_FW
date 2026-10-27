import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Relative base: the same build runs from any web path, inside Capacitor
// (iOS/Android) and as an installable web app.
export default defineConfig({
  base: "./",
  plugins: [react()],
  build: { outDir: "dist", sourcemap: true },
  test: { environment: "node", include: ["src/**/*.test.ts"] },
} as never);

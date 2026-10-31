/**
 * ACwing app protocol v2 – identifiers.
 *
 * Single source of truth for the app side. The firmware side is specified in
 * `HANDOVER_FW.md` (same folder as this app) and must use the same numbers.
 *
 * Transport (unchanged from phase 2, App/bsp/ble_tlm_svc.c):
 *   service 7E4A0001-…, TLM notify 7E4A0002-…, CMD write 7E4A0003-… (≤ 20 bytes)
 * Frames (unchanged, App/telemetry/telemetry.h):
 *   A5 5A | TYPE | LEN | PAYLOAD | CRC16-CCITT-FALSE (LSB, MSB), little-endian
 */

export const BLE = {
  service: "7e4a0001-2c6b-4f8e-9a5d-3b1c0e1d2f30",
  tlm: "7e4a0002-2c6b-4f8e-9a5d-3b1c0e1d2f30",
  cmd: "7e4a0003-2c6b-4f8e-9a5d-3b1c0e1d2f30",
  namePrefixes: ["ACw", "IEKF", "STM32WB"],
  cmdMaxLen: 20,
} as const;

export const PROTO_VERSION = 2;

/** Telemetry frame types, drone → app. 0x01–0x04 already exist in the firmware. */
export const FT = {
  STATE: 0x01, // 71 B, existing
  INIT: 0x02, // 137 B, existing
  STATUS: 0x03, // 62 B (54 B phase 1), existing
  REPLAY: 0x04, // USB only, ignored by the app
  CAPS: 0x05,
  FLIGHT: 0x06,
  POWER: 0x07,
  NAV: 0x08,
  MOTORS: 0x09,
  SYSTEM: 0x0a,
  EVENT: 0x0b,
  PARAM: 0x0c,
  CALIB: 0x0d,
  ACK: 0x0e,
  CHECKS: 0x0f,
} as const;

/** Command opcodes, app → drone, written on CMD. Format: [op][seq][payload…] (except legacy 0x01/0x02). */
export const OP = {
  SEND_INIT: 0x01, // legacy, [0x01]
  STATE_DECIM: 0x02, // legacy, [0x02, N]
  HEARTBEAT: 0x10,
  RC: 0x11,
  ARM: 0x12,
  DISARM: 0x13,
  TAKEOFF: 0x14,
  LAND: 0x15,
  RTH: 0x16,
  HOLD: 0x17,
  KILL: 0x18,
  SET_MODE: 0x19,
  PARAM_GET: 0x20,
  PARAM_SET: 0x21,
  PARAM_SAVE: 0x22,
  PARAM_DEFAULTS: 0x23,
  PARAM_LIST: 0x24,
  CAL_START: 0x30,
  CAL_CAPTURE: 0x31,
  CAL_ABORT: 0x32,
  CAL_SAVE: 0x33,
  CAL_RESTORE: 0x34,
  CAL_SET_REF: 0x35,
  MOTOR_TEST: 0x36,
  MOTOR_REVERSE: 0x37,
  MOTOR_REMAP: 0x38,
  STREAM_RATES: 0x40,
  REQUEST: 0x41,
  SET_NAME: 0x42,
} as const;

export const ARM_MAGIC = [0xa5, 0x5a] as const;
export const KILL_MAGIC = [0x4b, 0x49, 0x4c, 0x4c] as const; // "KILL"
export const PROPS_OFF_MAGIC = 0x50; // 'P'

/** ACK result codes */
export enum AckResult {
  OK = 0,
  REJECTED = 1,
  BUSY = 2,
  INVALID = 3,
  UNSUPPORTED = 4,
  IN_PROGRESS = 5,
}

/** ACK detail / rejection reasons */
export enum Reason {
  NONE = 0,
  NOT_LANDED = 1,
  CHECKS_FAILING = 2,
  THROTTLE_NOT_LOW = 3,
  NO_GPS = 4,
  BATTERY_LOW = 5,
  NO_HOME = 6,
  RADIO_OVERRIDE = 7,
  ARMED = 8,
  PROPS_CONFIRM_MISSING = 9,
  NOT_ARMED = 10,
  CALIBRATING = 11,
  NOT_CALIBRATING = 12,
  NOT_STILL = 13,
  OUT_OF_RANGE = 14,
  UNKNOWN_PARAM = 15,
  SENSOR_MISSING = 16,
}

export enum FlightMode {
  ACRO = 0,
  ANGLE = 1,
  ALT_HOLD = 2,
  POS_HOLD = 3,
}

export enum ArmState {
  DISARMED = 0,
  ARMED = 1,
}

export enum AutoAction {
  NONE = 0,
  TAKEOFF = 1,
  LAND = 2,
  RTH = 3,
  HOLD = 4,
  FAILSAFE = 5,
}

/** FLIGHT.failsafe bits */
export const FS = {
  APP_LINK: 1 << 0,
  RADIO: 1 << 1,
  BATTERY: 1 << 2,
  GEOFENCE: 1 << 3,
  GPS: 1 << 4,
  FLOW: 1 << 5,
} as const;

/** FLIGHT.flags bits */
export const FF = {
  IN_AIR: 1 << 0,
  LANDED: 1 << 1,
  HOME_SET: 1 << 2,
  GEOFENCE_ON: 1 << 3,
  BEGINNER: 1 << 4,
  ARM_BLOCKED: 1 << 5,
} as const;

export enum InputSource {
  NONE = 0,
  APP = 1,
  ELRS = 2,
}

/** CAPS.sensors bits */
export const SENSOR = {
  IMU: 1 << 0,
  BARO: 1 << 1,
  MAG_FC: 1 << 2,
  MAG_GPS: 1 << 3,
  GNSS: 1 << 4,
  FLOW: 1 << 5,
  TOF: 1 << 6,
  ESC_TLM: 1 << 7,
  CURRENT: 1 << 8,
  ELRS: 1 << 9,
  LEDS: 1 << 10,
  FLASH_LOG: 1 << 11,
} as const;

export enum Model {
  INDOOR = 0,
  OUTDOOR = 1,
}

export enum FrameType {
  QUAD_X = 0,
  HEXA_X = 1,
  OCTO_X = 2,
}

export enum BattWarn {
  OK = 0,
  LOW = 1,
  CRITICAL = 2,
  EMERGENCY = 3,
}

export enum EventLevel {
  INFO = 0,
  WARN = 1,
  ERROR = 2,
  CRITICAL = 3,
}

/** EVENT codes. The app translates known codes; unknown ones show the text sent by the drone. */
export enum EventCode {
  ARMED = 1,
  DISARMED = 2,
  TAKEOFF_STARTED = 3,
  LANDED = 4,
  MODE_CHANGED = 5,
  RTH_STARTED = 6,
  HOME_SET = 7,
  /** firmware extension: no lift-off within 6 s, back on the ground */
  TAKEOFF_ABORTED = 8,
  FAILSAFE_APP_LINK = 20,
  FAILSAFE_RADIO = 21,
  FAILSAFE_BATTERY = 22,
  GEOFENCE_BREACH = 23,
  GPS_LOST = 24,
  FLOW_LOST = 25,
  BATTERY_LOW = 30,
  BATTERY_CRITICAL = 31,
  BATTERY_EMERGENCY = 32,
  /** firmware extension (ERROR): the battery task stopped, battery unknown */
  BATTERY_MONITOR_LOST = 33,
  MAG_INTERFERENCE = 40,
  HIGH_VIBRATION = 41,
  MOTOR_HOT = 42,
  RADIO_OVERRIDE = 43,
  CALIB_DONE = 50,
  CALIB_FAILED = 51,
  PARAMS_SAVED = 60,
  MOTOR_KILLED = 70,
}

export enum CalType {
  ACCEL = 1,
  LEVEL = 2,
  GYRO = 3,
  COMPASS = 4,
  FLOW_TOF = 5,
  BATTERY = 6,
  MOTORS = 7,
  STICKS = 8,
}

export enum CalState {
  IDLE = 0,
  RUNNING = 1,
  WAITING_USER = 2,
  DONE = 3,
  FAILED = 4,
  SAVED = 5,
}

/** CHECKS item ids */
export enum CheckId {
  BATTERY = 1,
  IMU = 2,
  COMPASS = 3,
  GNSS = 4,
  HOME = 5,
  RADIO = 6,
  EKF = 7,
  FAILSAFE = 8,
  FLOW = 9,
  THROTTLE = 10,
  CALIBRATION = 11,
}

export enum CheckState {
  OK = 0,
  WARN = 1,
  FAIL = 2,
  NA = 3,
}

/** Drone-side parameters (PARAM_GET/SET, PARAM frame). Values are float32 on the wire. */
export interface ParamDef {
  id: number;
  key: string;
  def: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  /** enum options (value → i18n key) */
  options?: number[];
  /** sensor bits required for the parameter to be shown */
  needs?: number;
}

export const PARAMS: ParamDef[] = [
  { id: 1, key: "takeoffAlt", def: 1.0, min: 0.5, max: 3, step: 0.1, unit: "m" },
  { id: 2, key: "landSpeed", def: 0.7, min: 0.3, max: 1.5, step: 0.1, unit: "m/s" },
  { id: 3, key: "autoDisarm", def: 2, min: 1, max: 10, step: 1, unit: "s" },
  { id: 4, key: "maxTilt", def: 30, min: 10, max: 45, step: 1, unit: "°" },
  { id: 5, key: "maxYawRate", def: 120, min: 30, max: 360, step: 10, unit: "°/s" },
  { id: 6, key: "maxClimb", def: 2.0, min: 0.5, max: 5, step: 0.1, unit: "m/s" },
  { id: 7, key: "maxHSpeed", def: 8, min: 1, max: 15, step: 1, unit: "m/s" },
  { id: 8, key: "beginner", def: 0, min: 0, max: 1, step: 1 },
  { id: 10, key: "fsAppShort", def: 0, min: 0, max: 1, step: 1, options: [0, 1] },
  { id: 11, key: "fsAppLong", def: 2, min: 0, max: 2, step: 1, options: [2, 1, 0] },
  { id: 12, key: "fsRadio", def: 2, min: 0, max: 2, step: 1, options: [2, 1, 0] },
  { id: 13, key: "fsGpsLost", def: 0, min: 0, max: 2, step: 1, options: [0, 1, 2], needs: SENSOR.GNSS },
  { id: 14, key: "fsFlowLost", def: 0, min: 0, max: 1, step: 1, options: [0, 1], needs: SENSOR.FLOW },
  { id: 20, key: "battWarn", def: 3.6, min: 3.3, max: 3.9, step: 0.05, unit: "V" },
  { id: 21, key: "battCrit", def: 3.5, min: 3.2, max: 3.8, step: 0.05, unit: "V" },
  { id: 22, key: "battEmerg", def: 3.3, min: 3.0, max: 3.6, step: 0.05, unit: "V" },
  // firmware extensions (0.5.0): 0 = unknown capacity / cells detected automatically
  { id: 23, key: "battCapacity", def: 0, min: 0, max: 20000, step: 10, unit: "mAh", needs: SENSOR.CURRENT },
  { id: 24, key: "battCells", def: 0, min: 0, max: 6, step: 1, unit: "S", options: [0, 1, 2, 3, 4, 5, 6] },
  { id: 30, key: "gfMaxAlt", def: 30, min: 5, max: 120, step: 1, unit: "m" },
  { id: 31, key: "gfCeiling", def: 2.5, min: 1, max: 4, step: 0.1, unit: "m", needs: SENSOR.TOF },
  { id: 32, key: "gfMaxDist", def: 150, min: 20, max: 500, step: 10, unit: "m", needs: SENSOR.GNSS },
  { id: 33, key: "gfAction", def: 0, min: 0, max: 2, step: 1, options: [0, 1, 2] },
  { id: 40, key: "rthAlt", def: 30, min: 5, max: 60, step: 1, unit: "m", needs: SENSOR.GNSS },
  { id: 41, key: "rthStickCancel", def: 1, min: 0, max: 1, step: 1, needs: SENSOR.GNSS },
  { id: 50, key: "armRequireChecks", def: 1, min: 0, max: 1, step: 1 },
  { id: 60, key: "compassPrimary", def: 1, min: 0, max: 1, step: 1, options: [1, 0], needs: SENSOR.MAG_GPS },
  // firmware extension (0.5.0): flight log in the drone flash
  { id: 70, key: "logMode", def: 1, min: 0, max: 2, step: 1, options: [1, 2, 0], needs: SENSOR.FLASH_LOG },
];

export const PARAM_BY_KEY: Record<string, ParamDef> = Object.fromEntries(PARAMS.map((p) => [p.key, p]));
export const PARAM_BY_ID: Record<number, ParamDef> = Object.fromEntries(PARAMS.map((p) => [p.id, p]));

/** Failsafe/geofence action values used by the enum parameters */
export const ACTION = {
  fsAppShort: { 0: "hover", 1: "land" },
  fsAppLong: { 0: "hover", 1: "land", 2: "rth" },
  fsRadio: { 0: "hover", 1: "land", 2: "rth" },
  fsGpsLost: { 0: "useFlow", 1: "altHold", 2: "land" },
  fsFlowLost: { 0: "altHoldWarn", 1: "land" },
  gfAction: { 0: "stopBack", 1: "hover", 2: "rth" },
  compassPrimary: { 0: "compassFc", 1: "compassGps" },
  battCells: { 0: "auto" },
  logMode: { 0: "logOff", 1: "logArmed", 2: "logAlways" },
} as Record<string, Record<number, string>>;

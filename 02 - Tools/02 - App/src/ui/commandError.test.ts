import { describe, expect, it } from "vitest";
import { commandErrorToast } from "./commandError";
import { CommandError } from "../link/drone";
import { AckResult, Reason, EventCode, DisarmCause } from "../protocol/ids";
import { translate } from "../i18n";
import { eventText } from "../alerts/alerts";

const t = (k: string, v?: Record<string, string | number>) => translate("en", k, v);

describe("command refusal toast", () => {
  it("UNSUPPORTED with a reason says the reason (MOTOR_TEST, MOTORS_ENABLE = 0)", () => {
    const m = commandErrorToast(new CommandError(AckResult.UNSUPPORTED, Reason.MOTORS_DISABLED), t);
    expect(m.text).toBe(`Not done: ${t(`reasons.${Reason.MOTORS_DISABLED}`)}`);
  });
  it("REJECTED with a reason says it", () => {
    expect(commandErrorToast(new CommandError(AckResult.REJECTED, Reason.ARMED), t).text).toBe("Not done: the drone is armed");
  });
  it("no reason, or one this app does not know: the generic text", () => {
    expect(commandErrorToast(new CommandError(AckResult.UNSUPPORTED, Reason.NONE), t).text).toBe(t("cmdError.other"));
    expect(commandErrorToast(new CommandError(AckResult.REJECTED, 250), t).text).toBe(t("cmdError.other"));
  });
  it("timeout and offline", () => {
    expect(commandErrorToast(new CommandError("timeout", 0), t)).toEqual({ text: t("cmdError.timeout"), level: "error" });
    expect(commandErrorToast(new CommandError("offline", 0), t).text).toBe(t("cmdError.offline"));
  });
});

describe("DISARMED event text", () => {
  it("says why when the firmware tells (>= 0.16.3)", () => {
    expect(eventText({ code: EventCode.DISARMED, text: "", arg: DisarmCause.NO_TAKEOFF }, "en")).toBe("Disarmed: no take-off within 10 s");
    expect(eventText({ code: EventCode.DISARMED, text: "", arg: DisarmCause.APP_LINK }, "it")).toBe("Disarmato: collegamento con l’app perso");
  });
  it("a command, an older firmware or an unknown cause: plain 'Disarmed'", () => {
    expect(eventText({ code: EventCode.DISARMED, text: "", arg: DisarmCause.COMMAND }, "en")).toBe("Disarmed");
    expect(eventText({ code: EventCode.DISARMED, text: "", arg: 99 }, "en")).toBe("Disarmed");
  });
});

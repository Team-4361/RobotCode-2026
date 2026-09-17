package frc.robot.logics;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class ControlsLogger {
  private static final int BUTTON_COUNT = 16;
  private static final int AXIS_COUNT = 8;

  private final CommandJoystick left;
  private final CommandJoystick right;
  private final CommandXboxController operator;
  private final CommandXboxController test; // port 3; pass null if unused

  private final boolean[] leftPrev = new boolean[BUTTON_COUNT + 1];
  private final boolean[] rightPrev = new boolean[BUTTON_COUNT + 1];
  private final boolean[] operatorPrev = new boolean[BUTTON_COUNT + 1];
  private final boolean[] testPrev = new boolean[BUTTON_COUNT + 1];

  public ControlsLogger(
      CommandJoystick left,
      CommandJoystick right,
      CommandXboxController operator,
      CommandXboxController test) {
    this.left = left;
    this.right = right;
    this.operator = operator;
    this.test = test;
  }

  public void periodic() {
    logStick("Controls/DriverLeft", left.getHID(), leftPrev);
    logStick("Controls/DriverRight", right.getHID(), rightPrev);
    logXbox("Controls/Operator", operator, operatorPrev);
    if (test != null) {
      logXbox("Controls/Test", test, testPrev);
    }
  }

  private void logStick(String prefix, GenericHID hid, boolean[] prev) {
    Logger.recordOutput(prefix + "/Connected", hid.isConnected());
    Logger.recordOutput(prefix + "/X", hid.getRawAxis(0));
    Logger.recordOutput(prefix + "/Y", hid.getRawAxis(1));
    Logger.recordOutput(prefix + "/Twist", hid.getRawAxis(2));
    Logger.recordOutput(prefix + "/Throttle", hid.getRawAxis(3));
    for (int a = 0; a < AXIS_COUNT; a++) {
      Logger.recordOutput(prefix + "/Axis/" + a, hid.getRawAxis(a));
    }
    Logger.recordOutput(prefix + "/POV", hid.getPOV());
    logButtons(prefix, hid, prev);
  }

  private void logXbox(String prefix, CommandXboxController xbox, boolean[] prev) {
    XboxController hid = xbox.getHID();
    Logger.recordOutput(prefix + "/Connected", hid.isConnected());

    Logger.recordOutput(prefix + "/A", hid.getAButton());
    Logger.recordOutput(prefix + "/B", hid.getBButton());
    Logger.recordOutput(prefix + "/X", hid.getXButton());
    Logger.recordOutput(prefix + "/Y", hid.getYButton());
    Logger.recordOutput(prefix + "/LB", hid.getLeftBumperButton());
    Logger.recordOutput(prefix + "/RB", hid.getRightBumperButton());
    Logger.recordOutput(prefix + "/Back", hid.getBackButton());
    Logger.recordOutput(prefix + "/Start", hid.getStartButton());
    Logger.recordOutput(prefix + "/LStickBtn", hid.getLeftStickButton());
    Logger.recordOutput(prefix + "/RStickBtn", hid.getRightStickButton());

    Logger.recordOutput(prefix + "/LeftX", hid.getLeftX());
    Logger.recordOutput(prefix + "/LeftY", hid.getLeftY());
    Logger.recordOutput(prefix + "/RightX", hid.getRightX());
    Logger.recordOutput(prefix + "/RightY", hid.getRightY());
    Logger.recordOutput(prefix + "/LT", hid.getLeftTriggerAxis());
    Logger.recordOutput(prefix + "/RT", hid.getRightTriggerAxis());
    Logger.recordOutput(prefix + "/POV", hid.getPOV());

    // Catch D-pad + any extra HID buttons the named API misses
    logButtons(prefix, hid, prev);
  }

  private void logButtons(String prefix, GenericHID hid, boolean[] prev) {
    for (int b = 1; b <= BUTTON_COUNT; b++) {
      boolean now = hid.getRawButton(b);
      boolean pressed = now && !prev[b];
      boolean released = !now && prev[b];
      prev[b] = now;

      Logger.recordOutput(prefix + "/Button/" + b, now);
      Logger.recordOutput(prefix + "/JustPressed/" + b, pressed);
      Logger.recordOutput(prefix + "/JustReleased/" + b, released);
    }
  }
}
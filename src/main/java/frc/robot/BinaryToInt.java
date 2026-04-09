package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/** Converts the binary value of the buttons on the joysticks to an integer. */
public class BinaryToInt {
  /**
   * Converts the binary value of the buttons on the joysticks to an integer.
   *
   * @param boxRight
   * @param boxLeft
   * @return The integer value of the binary value of the buttons on the
   *         joysticks.
   */
  public static int getInt(CommandJoystick boxRight, CommandJoystick boxLeft) {
    return (boxRight.button(3).getAsBoolean() ? 1 << 2 : 0) // 1 << 2 is the same as 2^2
        + (boxRight.button(2).getAsBoolean() ? 1 << 1 : 0) // 1 << 1 is the same as 2^1
        + (boxRight.button(1).getAsBoolean() ? 1 << 0 : 0); // 1 << 0 is the same as 2^0
  }
}

package frc.robot.math;

public class Range {
  public static boolean inRange(double value, double range, double target) {
    return value - range < target || value + range > target;
  }
}

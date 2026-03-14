package frc.robot.math;

public class Range {
  public static boolean inRange(double value, double upperRange, double lowerRange, double target) {
    return value < target + upperRange && value > target - upperRange;
  }
}

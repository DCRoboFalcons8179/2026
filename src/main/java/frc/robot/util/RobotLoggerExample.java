// Copyright (c) 2026 DC RoboFalcons
// Example usage of RobotLogger

package frc.robot.util;

import frc.robot.util.RobotLogger.LogMetric;

/**
 * Example showing how to use RobotLogger with annotations and manual registration.
 *
 * <p>Add this to your Robot.java:
 *
 * <pre>
 * {@literal @}Override
 * public void robotInit() {
 *   // Start the logger
 *   RobotLogger.getInstance().start();
 * }
 *
 * {@literal @}Override
 * public void robotPeriodic() {
 *   // Update all logged values every cycle
 *   RobotLogger.getInstance().periodic();
 * }
 * </pre>
 *
 * <p>In your subsystems, you can use annotations:
 *
 * <pre>
 * public class Drive extends SubsystemBase {
 *   private double currentSpeed = 0.0;
 *
 *   public Drive() {
 *     // Register all annotated methods in this class
 *     RobotLogger.getInstance().registerAnnotatedMetrics(this);
 *   }
 *
 *   {@literal @}LogMetric(key = "Drive/Speed")
 *   public double getSpeed() {
 *     return currentSpeed;
 *   }
 *
 *   {@literal @}LogMetric(key = "Drive/LeftVelocity")
 *   public double getLeftVelocity() {
 *     return leftModule.getVelocity();
 *   }
 * }
 * </pre>
 *
 * <p>Or register metrics manually:
 *
 * <pre>
 * public Drive() {
 *   RobotLogger.getInstance().registerMetric("Drive/Speed", this::getSpeed);
 *   RobotLogger.getInstance().registerMetric("Drive/Position", () -> getPosition().getX());
 * }
 * </pre>
 */
public class RobotLoggerExample {

  // Example subsystem with annotated methods
  public static class ExampleSubsystem {
    private double speed = 0.0;
    private double temperature = 25.0;
    private boolean enabled = false;

    public ExampleSubsystem() {
      // Register all @LogMetric annotated methods
      RobotLogger.getInstance().registerAnnotatedMetrics(this);

      // Or register manually
      RobotLogger.getInstance().registerMetric("Example/ManualMetric", () -> speed * 2.0);
    }

    @LogMetric(key = "Example/Speed")
    public double getSpeed() {
      return speed;
    }

    @LogMetric(key = "Example/Temperature")
    public double getTemperature() {
      return temperature;
    }

    @LogMetric(key = "Example/Enabled")
    public boolean isEnabled() {
      return enabled; // Will be logged as 1.0 or 0.0
    }

    public void setSpeed(double speed) {
      this.speed = speed;
      // Log important state changes
      RobotLogger.getInstance().logEvent("Example/SpeedChanged", String.format("%.2f", speed));
    }
  }

  // Example of logging in commands
  public static void logCommandExample() {
    // Log when a command starts
    RobotLogger.getInstance().logEvent("Commands/AutoAlign", "Started");

    // Log progress
    RobotLogger.getInstance().logValue("Commands/AutoAlign/Progress", 0.75);

    // Log when complete
    RobotLogger.getInstance().logBoolean("Commands/AutoAlign/Success", true);
  }

  // Example of checking battery status
  public static void checkBatteryExample() {
    RobotLogger logger = RobotLogger.getInstance();

    if (logger.isBatteryLow()) {
      System.out.println("WARNING: Battery voltage is low!");
      logger.logEvent("Battery/Warning", "Low voltage detected");
    }

    double voltage = logger.getBatteryVoltage();
    System.out.println("Current battery voltage: " + voltage + "V");
  }
}

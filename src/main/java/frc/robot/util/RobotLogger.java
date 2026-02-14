// Copyright (c) 2026 DC RoboFalcons
// Use of this source code is governed by a BSD-style license.

package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized logging system for tracking robot metrics and battery voltage. Supports annotation-
 * based logging for easy integration.
 *
 * <p>Usage:
 *
 * <pre>
 * // In Robot.java or RobotContainer.java
 * RobotLogger.getInstance().start();
 *
 * // Register custom metrics
 * RobotLogger.getInstance().registerMetric("Drive/Speed", () -> drive.getSpeed());
 *
 * // Or use annotations in your subsystem:
 * {@literal @}LogMetric(key = "Drive/CurrentSpeed")
 * public double getCurrentSpeed() {
 *   return speed;
 * }
 * </pre>
 */
public class RobotLogger {
  private static RobotLogger instance;
  private final Map<String, DoubleSupplier> metrics = new HashMap<>();
  private final PowerDistribution pdh;
  private boolean enabled = false;
  private double startTime = 0.0;

  /** Annotation to mark methods that should be automatically logged. */
  @Retention(RetentionPolicy.RUNTIME)
  @Target(ElementType.METHOD)
  public @interface LogMetric {
    /** The key path for the logged value (e.g., "Drive/Speed") */
    String key();

    /** Optional: Log frequency in seconds. Default is every cycle (0.02s) */
    double frequency() default 0.0;
  }

  private RobotLogger() {
    try {
      pdh = new PowerDistribution();
    } catch (Exception e) {
      System.err.println("Failed to initialize PowerDistribution: " + e.getMessage());
      throw new RuntimeException(e);
    }
  }

  /** Get the singleton instance of RobotLogger. */
  public static RobotLogger getInstance() {
    if (instance == null) {
      instance = new RobotLogger();
    }
    return instance;
  }

  /** Start logging. Call this in robotInit() or robotPeriodic(). */
  public void start() {
    enabled = true;
    startTime = Timer.getFPGATimestamp();
    System.out.println("========================================");
    System.out.println("RobotLogger STARTED - Battery logging active");
    System.out.println("Look for keys: Battery/Voltage, PDH/TotalCurrent, Robot/Uptime");
    System.out.println("========================================");
  }

  /** Stop logging. */
  public void stop() {
    enabled = false;
    System.out.println("RobotLogger stopped");
  }

  /**
   * Register a custom metric to be logged every cycle.
   *
   * @param key The logging key (e.g., "Drive/Speed")
   * @param supplier A supplier that returns the current value
   */
  public void registerMetric(String key, DoubleSupplier supplier) {
    metrics.put(key, supplier);
    System.out.println("Registered metric: " + key);
  }

  /**
   * Unregister a metric.
   *
   * @param key The logging key to remove
   */
  public void unregisterMetric(String key) {
    metrics.remove(key);
  }

  /**
   * Update all logged values. Call this in robotPeriodic() or in your periodic methods. This is
   * automatically called if you use the start() method.
   */
  public void periodic() {
    if (!enabled) {
      return;
    }

    // Log battery voltage
    double batteryVoltage = RobotController.getBatteryVoltage();
    Logger.recordOutput("RobotLogger/Battery/Voltage", batteryVoltage);
    Logger.recordOutput("RobotLogger/Battery/Voltage3v3", RobotController.getVoltage3V3());
    Logger.recordOutput("RobotLogger/Battery/Voltage5v", RobotController.getVoltage5V());
    Logger.recordOutput("RobotLogger/Battery/Current3v3", RobotController.getCurrent3V3());
    Logger.recordOutput("RobotLogger/Battery/Current5v", RobotController.getCurrent5V());
    Logger.recordOutput("RobotLogger/Battery/Current6v", RobotController.getCurrent6V());

    // Log brownout status
    Logger.recordOutput("RobotLogger/Battery/BrownedOut", RobotController.isBrownedOut());
    Logger.recordOutput("RobotLogger/Battery/SystemActive", RobotController.isSysActive());

    // Log PDH metrics if available
    try {
      Logger.recordOutput("RobotLogger/PDH/TotalCurrent", pdh.getTotalCurrent());
      Logger.recordOutput("RobotLogger/PDH/TotalPower", pdh.getTotalPower());
      Logger.recordOutput("RobotLogger/PDH/TotalEnergy", pdh.getTotalEnergy());
      Logger.recordOutput("RobotLogger/PDH/Voltage", pdh.getVoltage());
      Logger.recordOutput("RobotLogger/PDH/Temperature", pdh.getTemperature());
    } catch (Exception e) {
      // PDH might not be available in simulation
    }

    // Log uptime
    double uptime = Timer.getFPGATimestamp() - startTime;
    Logger.recordOutput("RobotLogger/Uptime", uptime);

    // Log all registered metrics
    for (Map.Entry<String, DoubleSupplier> entry : metrics.entrySet()) {
      try {
        double value = entry.getValue().getAsDouble();
        Logger.recordOutput(entry.getKey(), value);
      } catch (Exception e) {
        System.err.println("Error logging metric " + entry.getKey() + ": " + e.getMessage());
      }
    }
  }

  /**
   * Log a one-time event or state change.
   *
   * @param key The logging key
   * @param value The value to log
   */
  public void logEvent(String key, String value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Log a one-time numeric value.
   *
   * @param key The logging key
   * @param value The value to log
   */
  public void logValue(String key, double value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Log a one-time boolean value.
   *
   * @param key The logging key
   * @param value The value to log
   */
  public void logBoolean(String key, boolean value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Register all methods annotated with @LogMetric in the given object.
   *
   * @param object The object to scan for annotated methods
   */
  public void registerAnnotatedMetrics(Object object) {
    Class<?> clazz = object.getClass();
    for (var method : clazz.getDeclaredMethods()) {
      if (method.isAnnotationPresent(LogMetric.class)) {
        LogMetric annotation = method.getAnnotation(LogMetric.class);
        String key = annotation.key();

        // Validate method signature
        if (method.getParameterCount() != 0) {
          System.err.println(
              "Warning: @LogMetric method "
                  + method.getName()
                  + " must have no parameters. Skipping.");
          continue;
        }

        Class<?> returnType = method.getReturnType();
        if (returnType != double.class
            && returnType != int.class
            && returnType != float.class
            && returnType != long.class
            && returnType != boolean.class) {
          System.err.println(
              "Warning: @LogMetric method "
                  + method.getName()
                  + " must return a numeric or boolean type. Skipping.");
          continue;
        }

        // Create supplier that invokes the method
        DoubleSupplier supplier =
            () -> {
              try {
                Object result = method.invoke(object);
                if (result instanceof Number) {
                  return ((Number) result).doubleValue();
                } else if (result instanceof Boolean) {
                  return ((Boolean) result) ? 1.0 : 0.0;
                }
                return 0.0;
              } catch (Exception e) {
                System.err.println("Error invoking @LogMetric method " + method.getName());
                return 0.0;
              }
            };

        registerMetric(key, supplier);
      }
    }
  }

  /**
   * Get the current battery voltage.
   *
   * @return Battery voltage in volts
   */
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  /**
   * Check if battery voltage is critically low.
   *
   * @return true if voltage is below 11.5V
   */
  public boolean isBatteryLow() {
    return getBatteryVoltage() < 11.5;
  }

  /**
   * Get the robot uptime since logger was started.
   *
   * @return Uptime in seconds
   */
  public double getUptime() {
    return Timer.getFPGATimestamp() - startTime;
  }
}

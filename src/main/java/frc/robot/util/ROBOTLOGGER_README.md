# RobotLogger - Battery & Metrics Tracking System

A comprehensive logging system for tracking robot metrics, battery voltage, and power distribution automatically.

## Features

- ✅ **Automatic Battery Monitoring**: Tracks voltage, current, brownout status
- ✅ **Power Distribution Logging**: PDH current, power, energy, temperature
- ✅ **Annotation-Based Metrics**: Use `@LogMetric` for easy integration
- ✅ **Manual Registration**: Register custom metrics with lambda functions
- ✅ **One-Time Event Logging**: Log important state changes
- ✅ **Uptime Tracking**: Monitor how long the robot has been running
- ✅ **Low Battery Warnings**: Built-in battery health checking

## Quick Start

### 1. Already Integrated!

The `RobotLogger` is already integrated into `Robot.java`:
- Started in `robotInit()`
- Updated every cycle in `robotPeriodic()`

### 2. Add Metrics to Your Subsystems

#### Option A: Using Annotations (Recommended)

```java
public class Drive extends SubsystemBase {
  private double currentSpeed = 0.0;

  public Drive() {
    // Register all @LogMetric annotated methods
    RobotLogger.getInstance().registerAnnotatedMetrics(this);
  }

  @LogMetric(key = "Drive/Speed")
  public double getSpeed() {
    return currentSpeed;
  }

  @LogMetric(key = "Drive/Voltage")
  public double getVoltage() {
    return motor.getVoltage();
  }

  @LogMetric(key = "Drive/IsMoving")
  public boolean isMoving() {
    return Math.abs(currentSpeed) > 0.1;
  }
}
```

#### Option B: Manual Registration

```java
public class Shooter extends SubsystemBase {
  public Shooter() {
    RobotLogger logger = RobotLogger.getInstance();

    // Register with method references
    logger.registerMetric("Shooter/Speed", this::getSpeed);
    logger.registerMetric("Shooter/Temperature", this::getTemperature);

    // Register with lambdas
    logger.registerMetric("Shooter/IsReady", () -> isAtSpeed() ? 1.0 : 0.0);
  }
}
```

### 3. Log Important Events

```java
// In commands or subsystems
public void startShooting() {
  RobotLogger.getInstance().logEvent("Shooter/Action", "Started shooting");
  // ... your code
}

public void setSpeed(double speed) {
  this.targetSpeed = speed;
  RobotLogger.getInstance().logValue("Shooter/TargetSpeed", speed);
}
```

### 4. Check Battery Health

```java
RobotLogger logger = RobotLogger.getInstance();

if (logger.isBatteryLow()) {
  System.out.println("WARNING: Battery low!");
  // Maybe disable high-power features
}

double voltage = logger.getBatteryVoltage();
```

## What Gets Logged Automatically

Without any extra code, `RobotLogger` tracks:

### Battery Metrics
- `Battery/Voltage` - Main battery voltage
- `Battery/Voltage3v3` - 3.3V rail voltage
- `Battery/Voltage5v` - 5V rail voltage
- `Battery/Current3v3` - 3.3V rail current
- `Battery/Current5v` - 5V rail current
- `Battery/Current6v` - 6V rail current
- `Battery/BrownedOut` - Whether robot is in brownout
- `Battery/SystemActive` - System active status

### Power Distribution Hub (PDH)
- `PDH/TotalCurrent` - Total current draw
- `PDH/TotalPower` - Total power consumption
- `PDH/TotalEnergy` - Cumulative energy used
- `PDH/Voltage` - PDH voltage reading
- `PDH/Temperature` - PDH temperature

### Robot Status
- `Robot/Uptime` - Seconds since robot enabled

## Advanced Usage

### Conditional Logging

```java
@LogMetric(key = "Drive/DetailedMetric", frequency = 0.1)
public double getDetailedMetric() {
  return expensiveCalculation(); // Only called every 0.1s
}
```

### Unregister Metrics

```java
// Stop logging a metric
RobotLogger.getInstance().unregisterMetric("Drive/OldMetric");
```

### Stop/Start Logger

```java
// Temporarily pause logging
RobotLogger.getInstance().stop();

// Resume logging
RobotLogger.getInstance().start();
```

## Viewing Logs

All data is logged through AdvantageKit's `Logger`, so:

1. **View in AdvantageScope**: Open your log files in AdvantageScope
2. **Network Tables**: Data is published to NT4 for live viewing
3. **Search for**: Look for keys like `Battery/Voltage`, `Drive/Speed`, etc.

## Performance Notes

- Very lightweight - uses existing AdvantageKit infrastructure
- Metrics are only calculated when accessed
- No performance impact if not registered
- Safe to use in real-time loops

## Example Integration

See `RobotLoggerExample.java` for complete examples of:
- Subsystem integration
- Command logging
- Battery monitoring
- Custom metrics

## API Reference

| Method | Description |
|--------|-------------|
| `getInstance()` | Get singleton instance |
| `start()` | Enable logging |
| `stop()` | Disable logging |
| `periodic()` | Update all metrics (called automatically) |
| `registerMetric(key, supplier)` | Register a metric manually |
| `registerAnnotatedMetrics(object)` | Scan for @LogMetric annotations |
| `unregisterMetric(key)` | Remove a registered metric |
| `logEvent(key, value)` | Log a one-time string event |
| `logValue(key, value)` | Log a one-time number |
| `logBoolean(key, value)` | Log a one-time boolean |
| `getBatteryVoltage()` | Get current battery voltage |
| `isBatteryLow()` | Check if voltage < 11.5V |
| `getUptime()` | Get robot uptime in seconds |

## Troubleshooting

**Metrics not showing up?**
- Make sure you called `registerAnnotatedMetrics(this)` in constructor
- Check that method is annotated with `@LogMetric`
- Verify method has no parameters and returns a number/boolean

**PDH errors in simulation?**
- Normal - PDH is not available in sim, errors are caught automatically

**Battery voltage reads 0?**
- Check that RobotLogger was started in `robotInit()`
- May not work in replay mode

---

Created for DC RoboFalcons 8179 - 2026 Season

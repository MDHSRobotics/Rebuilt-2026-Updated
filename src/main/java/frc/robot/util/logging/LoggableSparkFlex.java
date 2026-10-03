package frc.robot.util.logging;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.hardware.bus.CANPort;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * A SparkFlex wrapper that automatically publishes motor data to NetworkTables.
 *
 * <p>Declare which values you want logged using {@link LoggedValue}, then call {@link #updateAll()}
 * from robotPeriodic() to flush everything at once.
 *
 * <h3>Example usage:</h3>
 *
 * <pre>{@code
 * private final LoggableSparkFlex m_shooterMotor = new LoggableSparkFlex(
 *     9, MotorType.kBrushless, m_table, "Left Motor",
 *     EncoderType.RELATIVE, LoggedValue.VELOCITY, LoggedValue.TEMPERATURE
 * );
 *
 * // In robotPeriodic():
 * LoggableSparkFlex.updateAll();
 *
 * // Setting a velocity setpoint — also updates the target publisher automatically:
 * m_shooterMotor.setVelocity(3000);
 *
 * // In a test command:
 * m_shooterMotor.setTestResult(rpm > ShooterConstants.TEST_RPM);
 * m_shooterMotor.resetTestResult(); // back to false between tests
 * }</pre>
 */
public class LoggableSparkFlex extends SparkFlex {

  /** The values a LoggableSparkFlex can automatically publish to NT. */
  public enum LoggedValue {
    VELOCITY, // current + target RPM
    POSITION, // current + target position (rotations)
    CURRENT, // output current (amps)
    TEMPERATURE, // motor temperature (°C)
    OUTPUT_VOLTAGE // applied output * bus voltage
  }

  /** Whether to use the built-in relative encoder or an attached absolute encoder. */
  public enum EncoderType {
    RELATIVE,
    ABSOLUTE
  }

  // ── Static registry ───────────────────────────────────────────────────────

  private static final List<LoggableSparkFlex> instances = new ArrayList<>();

  /** Call once from robotPeriodic() to update all registered motors. */
  public static void updateAll() {
    for (LoggableSparkFlex motor : instances) {
      motor.update();
    }
  }

  // ── Instance state ────────────────────────────────────────────────────────

  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController controller;
  private final Set<LoggedValue> loggedValues;

  // Publishers — only created if the corresponding value is being logged
  private DoublePublisher velocityPub, targetVelocityPub;
  private DoublePublisher positionPub, targetPositionPub;
  private DoublePublisher currentPub;
  private DoublePublisher temperaturePub;
  private DoublePublisher outputVoltagePub;

  // Test result — always created, lives under Test/<motorName>_OK
  private final BooleanPublisher testResultPub;

  // Tracked setpoints so we can publish them
  private double targetVelocity = 0;
  private double targetPosition = 0;

  // ── Constructor ───────────────────────────────────────────────────────────

  /**
   * @param canPort Port - see CANPort.java for available ports
   * @param canId CAN ID of the motor
   * @param motorType brushless or brushed
   * @param parentTable the subsystem's NT table (e.g. m_table)
   * @param motorName name for this motor's subtable (e.g. "Left Motor")
   * @param encoderType whether to use the relative or absolute encoder
   * @param valuesToLog which values to publish (varargs, pick any combination)
   */
  public LoggableSparkFlex(
      CANPort bus,
      int canId,
      MotorType motorType,
      NetworkTable parentTable,
      String motorName,
      EncoderType encoderType,
      LoggedValue... valuesToLog) {
    super(bus, canId, motorType);

    if (encoderType == EncoderType.ABSOLUTE) {
      absoluteEncoder = getAbsoluteEncoder();
      relativeEncoder = null;
    } else {
      relativeEncoder = getEncoder();
      absoluteEncoder = null;
    }

    this.controller = getClosedLoopController();
    this.loggedValues =
        valuesToLog.length > 0
            ? EnumSet.copyOf(List.of(valuesToLog))
            : EnumSet.noneOf(LoggedValue.class);

    // Motor data lives under the subsystem table e.g. Shooter/Left Motor/
    NetworkTable table = parentTable.getSubTable(motorName);

    if (logs(LoggedValue.VELOCITY)) {
      velocityPub = table.getDoubleTopic("Current Velocity").publish();
      targetVelocityPub = table.getDoubleTopic("Target Velocity").publish();
    }
    if (logs(LoggedValue.POSITION)) {
      positionPub = table.getDoubleTopic("Current Position").publish();
      targetPositionPub = table.getDoubleTopic("Target Position").publish();
    }
    if (logs(LoggedValue.CURRENT)) {
      currentPub = table.getDoubleTopic("Output Current").publish();
    }
    if (logs(LoggedValue.TEMPERATURE)) {
      temperaturePub = table.getDoubleTopic("Temperature").publish();
    }
    if (logs(LoggedValue.OUTPUT_VOLTAGE)) {
      outputVoltagePub = table.getDoubleTopic("Output Voltage").publish();
    }

    // Test result lives under Test/<motorName>_OK — always created
    testResultPub =
        NetworkTableInstance.getDefault()
            .getTable("Test")
            .getBooleanTopic(motorName + "_OK")
            .publish();
    testResultPub.set(false);

    instances.add(this);
  }

  // ── Setpoint methods ──────────────────────────────────────────────────────

  /**
   * Set a velocity setpoint (RPM) via closed-loop control. Automatically updates the target
   * velocity publisher.
   */
  public void setVelocity(double rpm) {
    targetVelocity = rpm;
    controller.setSetpoint(rpm, ControlType.kVelocity);
  }

  /**
   * Set a position setpoint (rotations) via closed-loop control. Automatically updates the target
   * position publisher.
   */
  public void setPosition(double rotations) {
    targetPosition = rotations;
    controller.setSetpoint(rotations, ControlType.kPosition);
  }

  // ── Getter methods ────────────────────────────────────────────────────────

  public double getVelocity() {
    return absoluteEncoder != null
        ? absoluteEncoder.getVelocity().get(0.0)
        : relativeEncoder.getVelocity().get(0.0);
  }

  public double getPosition() {
    return absoluteEncoder != null
        ? absoluteEncoder.getPosition().get(0.0)
        : relativeEncoder.getPosition().get(0.0);
  }
  // ── Test result methods ───────────────────────────────────────────────────

  /** Publish a pass/fail result for this motor to the Test table. */
  public void setTestResult(boolean passed) {
    testResultPub.set(passed);
  }

  /** Reset this motor's test result back to false. Call from resetTestIndicators(). */
  public void resetTestResult() {
    testResultPub.set(false);
  }

  // ── Internal ──────────────────────────────────────────────────────────────

  private void update() {

    double currentVelocity =
        absoluteEncoder != null
            ? absoluteEncoder.getVelocity().get(0.0)
            : relativeEncoder.getVelocity().get(0.0);

    double currentPosition =
        absoluteEncoder != null
            ? absoluteEncoder.getPosition().get(0.0)
            : relativeEncoder.getPosition().get(0.0);

    if (logs(LoggedValue.VELOCITY)) {
      velocityPub.set(currentVelocity);
      targetVelocityPub.set(targetVelocity);
    }
    if (logs(LoggedValue.POSITION)) {
      positionPub.set(currentPosition);
      targetPositionPub.set(targetPosition);
    }
    if (logs(LoggedValue.CURRENT)) {
      currentPub.set(getOutputCurrent().get(0.));
    }
    if (logs(LoggedValue.TEMPERATURE)) {
      temperaturePub.set(getMotorTemperature().get(0.));
    }
    if (logs(LoggedValue.OUTPUT_VOLTAGE)) {
      outputVoltagePub.set(getBusVoltage().get(0.) * getAppliedOutput().get(0.));
    }
  }

  private boolean logs(LoggedValue value) {
    return loggedValues.contains(value);
  }
}

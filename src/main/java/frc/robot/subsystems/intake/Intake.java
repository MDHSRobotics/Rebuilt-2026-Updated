package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

import frc.robot.Constants.BusConstants;
import frc.robot.util.Testable;
import frc.robot.util.logging.LoggableSparkFlex;
import frc.robot.util.logging.LoggableSparkFlex.EncoderType;
import frc.robot.util.logging.LoggableSparkFlex.LoggedValue;
import frc.robot.Constants;
  
public class Intake extends SubsystemBase implements Testable {

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Intake");

  // Motors
  private final LoggableSparkFlex m_intakeRightMotor =
      new LoggableSparkFlex(
          BusConstants.INTAKE_BUS,
          IntakeConstants.INTAKE_RIGHT_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Right Arm",
          EncoderType.RELATIVE,
          LoggedValue.POSITION,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE);
  private final LoggableSparkFlex m_intakeLeftMotor =
      new LoggableSparkFlex(
          BusConstants.INTAKE_BUS,
          IntakeConstants.INTAKE_LEFT_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Left Arm",
          EncoderType.RELATIVE,
          LoggedValue.POSITION,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE);
  private final LoggableSparkFlex m_spinnerMotor =
      new LoggableSparkFlex(
          BusConstants.INTAKE_BUS,
          IntakeConstants.INTAKE_SPINNERS_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Spinner",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY);

  private boolean deployed = false;

  public Intake() {
    SparkFlexConfig intakeRightConfig = new SparkFlexConfig();
    intakeRightConfig
        .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    intakeRightConfig
        .signals
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true);
    intakeRightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(IntakeConstants.K_P);
    m_intakeRightMotor.configure(
        intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
    intakeLeftConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    intakeLeftConfig
        .signals
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true);
    intakeLeftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(IntakeConstants.K_P);
    m_intakeLeftMotor.configure(
        intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeSpinnerConfig = new SparkFlexConfig();
    intakeSpinnerConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kCoast);
    m_spinnerMotor.configure(
        intakeSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void stopMotors() {
    m_intakeRightMotor.stopMotor();
    m_intakeLeftMotor.stopMotor();
    m_spinnerMotor.stopMotor();
  }

  public void runMotors(double leftIntakeMotorPower, double rightIntakeMotorPower) {
    m_intakeRightMotor.setThrottle(leftIntakeMotorPower);
    m_intakeLeftMotor.setThrottle(rightIntakeMotorPower);
  }

  public void runSpinner(double power) {
    m_spinnerMotor.setThrottle(power);
    if (power > 0) {
      runMotors(IntakeConstants.INTAKE_MOTORS_DOWN_POWER, IntakeConstants.INTAKE_MOTORS_DOWN_POWER);
    }
  }

  public void deployedPosition() {
    deployed = true;
    m_intakeLeftMotor.setPosition(IntakeConstants.PICKUP_POSITION_LEFT);
    m_intakeRightMotor.setPosition(IntakeConstants.PICKUP_POSITION_RIGHT);
  }

  public void stowedPosition() {
    deployed = false;
    m_intakeLeftMotor.setPosition(IntakeConstants.STOWED_POSITION_LEFT);
    m_intakeRightMotor.setPosition(IntakeConstants.STOWED_POSITION_RIGHT);
  }

  public void stopSpinnerMotors() {
    m_spinnerMotor.stopMotor();
  }

  public boolean isDeployed() {
    return deployed;
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_spinnerMotor.setThrottle(IntakeConstants.TEST_POWER);
                  double rpm = m_spinnerMotor.getVelocity();
                  m_spinnerMotor.setTestResult(rpm > IntakeConstants.TEST_RPM);
                },
                this)
            .withTimeout(IntakeConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_spinnerMotor.setVelocity(0.0);
                  m_spinnerMotor.setTestResult(Math.abs(m_spinnerMotor.getVelocity()) < 5);
                },
                this)
            .withTimeout(IntakeConstants.TEST_TIMEOUT));
  }

  public void resetTestIndicators() {
    m_spinnerMotor.resetTestResult();
    m_intakeLeftMotor.resetTestResult();
    m_intakeRightMotor.resetTestResult();
  }
}

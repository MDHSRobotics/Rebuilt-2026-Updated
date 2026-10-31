package frc.robot.subsystems.hopper;

import static frc.robot.util.EpsilonEquals.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

import frc.robot.Constants.BusConstants;
import frc.robot.subsystems.hopper.HopperConstants.HopperPowers;
import frc.robot.util.Testable;
import frc.robot.util.logging.LoggableSparkFlex;
import frc.robot.util.logging.LoggableSparkFlex.EncoderType;
import frc.robot.util.logging.LoggableSparkFlex.LoggedValue;

public class Hopper extends SubsystemBase implements Testable {

  private final TelemetryTable m_hopperTelemetry = Telemetry.getTable("Hopper");

  private final LoggableSparkFlex m_hopperMotor =
      new LoggableSparkFlex(
          BusConstants.HOPPER_BUS,
          HopperConstants.HOPPER_MOTOR_ID,
          MotorType.kBrushless,
          m_hopperTelemetry,
          "Rollers",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE);
    
  private boolean m_intaking = false;
  private boolean m_shooting = false;
  private boolean m_reversing = false;

  public Hopper() {
    SparkFlexConfig m_hopperConfig = new SparkFlexConfig();
    m_hopperConfig.smartCurrentLimit(HopperConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_hopperConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(HopperConstants.K_P);
    m_hopperMotor.configure(
        m_hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runHopper(HopperConstants.HopperPowers powerEnum) {
    m_hopperMotor.setThrottle(powerEnum.power);
  }

  public void stopMotors() {
    m_hopperMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (m_shooting) {
      runHopper(HopperPowers.SHOOT);
    } else if (m_intaking) {
      runHopper(HopperPowers.INTAKE);
    } else if (m_reversing) {
      runHopper(HopperPowers.INTAKE_REVERSE);
    } else {
      stopMotors();
    }

    m_hopperTelemetry.log("Shooting", m_shooting);
    m_hopperTelemetry.log("Intaking", m_intaking);
    m_hopperTelemetry.log("Reversing", m_reversing);

  }

  public void setIntaking(boolean enabled) {
    m_intaking = enabled;
  }

  public void setShooting(boolean enabled) {
    m_shooting = enabled;
  }

  public void setReversing(boolean enabled) {
    m_reversing = enabled;
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_hopperMotor.setThrottle(HopperConstants.TEST_POWER);
                  double rpm = m_hopperMotor.getVelocity();
                  m_hopperMotor.setTestResult(rpm > HopperConstants.TEST_RPM);
                },
                this)
            .withTimeout(HopperConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_hopperMotor.setVelocity(0.0);
                  m_hopperMotor.setTestResult(epsilonEquals(m_hopperMotor.getVelocity(), 0.0, 5.0));
                },
                this)
            .withTimeout(HopperConstants.TEST_TIMEOUT));
  }

  public void resetTestIndicators() {
    m_hopperMotor.resetTestResult();
  }
}

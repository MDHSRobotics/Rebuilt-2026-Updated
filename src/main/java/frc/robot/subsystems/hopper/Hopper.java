package frc.robot.subsystems.hopper;

import static frc.robot.util.EpsilonEquals.*;

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

public class Hopper extends SubsystemBase implements Testable {

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Hopper");

  private final LoggableSparkFlex m_hopperMotor =
      new LoggableSparkFlex(
          BusConstants.HOPPER_BUS,
          HopperConstants.HOPPER_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Rollers",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE);

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
  public void periodic() {}

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

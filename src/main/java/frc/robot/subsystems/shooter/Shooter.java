package frc.robot.subsystems.shooter;

import static frc.robot.util.EpsilonEquals.epsilonEquals;

import com.limelightvision.Limelight;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.BusConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Aiming;
import frc.robot.util.PolynomialInterpolation;
import frc.robot.util.Testable;
import frc.robot.util.logging.LoggableSparkFlex;
import frc.robot.util.logging.LoggableSparkFlex.EncoderType;
import frc.robot.util.logging.LoggableSparkFlex.LoggedValue;
import frc.robot.util.logging.LoggedTunableNumber;

public class Shooter extends SubsystemBase implements Testable {

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Shooter");

  private final Limelight m_frontLimelight = new Limelight(Constants.VisionConstants.FRONT_LIMELIGHT_NAME);

  /* Spark Flex Motors */
  private final LoggableSparkFlex m_shooterLeftMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.SHOOTER_LEFT_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Left Shooter",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE);
  private final LoggableSparkFlex m_shooterRightMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.SHOOTER_RIGHT_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Right Shooter",
          EncoderType.RELATIVE);
  private final LoggableSparkFlex m_kickerMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.KICKER_MOTOR_ID,
          MotorType.kBrushless,
          m_table,
          "Kicker",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY);

  /* Networktables Publishers */
  private final DoublePublisher m_distanceRobotToTagPub =
      m_table.getDoubleTopic("Distance From Robot to AprilTag ").publish();
  private final DoublePublisher m_txAdjustmentPub =
      m_table.getDoubleTopic("TX Adjustment").publish();

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_P_SHOOTER);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", ShooterConstants.K_I_SHOOTER);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", ShooterConstants.K_D_SHOOTER);

  private double m_currentDistance = 0;
  private double m_lastDistance = 0;

  private double m_shooterTrim = 25;
  private boolean m_tagIsSeen = false;

  SparkFlexConfig shooterLeftMotorConfig;

  private PolynomialInterpolation polynomial =
      new PolynomialInterpolation(ShooterConstants.DISTANCES, ShooterConstants.RPMS);

  public Shooter() {

    // Left Motor Configurations
    shooterLeftMotorConfig = new SparkFlexConfig();
    shooterLeftMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP.get(), kI.get(), kD.get());
    m_shooterLeftMotor.configure(
        shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right Motor Configurations
    SparkFlexConfig shooterRightMotorConfig = new SparkFlexConfig();
    shooterRightMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP.get(), kI.get(), kD.get());
    m_shooterRightMotor.configure(
        shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Kicker Motor Configurations
    SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
    kickerMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_kickerMotor.configure(
        kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    // if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
    //   shooterLeftMotorConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    //   m_shooterLeftMotor.configure(
    //       shooterLeftMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // }

    m_currentDistance =
        Aiming.calculateTagDistance(
            VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE_INCHES,
            FieldConstants.DISTANCE_FROM_FLOOR_TO_HUB_TAG,
            VisionConstants.LIMELIGHT_MOUNT_ANGLE,
            m_frontLimelight.getTYDegrees());
    if (m_frontLimelight.hasTarget()) {
      m_tagIsSeen = true;
      m_lastDistance = m_currentDistance;
    } else {
      m_tagIsSeen = false;
      m_currentDistance = m_lastDistance;
    }

    m_distanceRobotToTagPub.set(m_currentDistance);
    Telemetry.log("AprilTag is recognized", m_tagIsSeen);
  }

  public void runLeftMotor(double power, double kickerPower) {
    m_shooterLeftMotor.setThrottle(power);
    m_kickerMotor.setThrottle(kickerPower);
  }

  public void stopMotors() {
    m_shooterLeftMotor.stopMotor();
    m_shooterRightMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public void shootBall() {
    rampUpShooter();
    m_kickerMotor.setThrottle(ShooterConstants.KICKER_SPEED);
  }

  public void shootBall(double rpm) {
    rampUpShooter(rpm);
    m_kickerMotor.setThrottle(ShooterConstants.KICKER_SPEED);
  }

  public void changeTrim(double amount) {
    m_shooterTrim += amount;
  }

  public void rampUpShooter() {
    double targetRPM =
        Aiming.calculateShooterRPM(
            polynomial, m_currentDistance);
    targetRPM += m_shooterTrim;
    m_shooterLeftMotor.setVelocity(targetRPM);
  }

  public void rampUpShooter(double rpm) {
    m_shooterLeftMotor.setVelocity(rpm);
  }

  public double getYawRotationalRate() {
    double txAdjusment = Aiming.getYawTxAdjustment(m_frontLimelight.getTXDegrees());
    m_txAdjustmentPub.set(txAdjusment);
    return txAdjusment;
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_kickerMotor.setThrottle(ShooterConstants.TEST_POWER);
                  double rpm = m_kickerMotor.getVelocity();
                  m_kickerMotor.setTestResult(rpm > ShooterConstants.TEST_RPM);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_kickerMotor.stopMotor();
                  ;
                  m_kickerMotor.setTestResult(Math.abs(m_kickerMotor.getVelocity()) < 5);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        new WaitCommand(ShooterConstants.TEST_TIMEOUT / 4),
        Commands.run(
                () -> {
                  m_shooterLeftMotor.setVelocity(ShooterConstants.TEST_RPM_2);
                  m_shooterLeftMotor.setTestResult(
                      epsilonEquals(
                          m_shooterLeftMotor.getVelocity(), ShooterConstants.TEST_RPM_2, 200));
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_shooterLeftMotor.setVelocity(0);
                  m_shooterLeftMotor.setTestResult(Math.abs(m_shooterLeftMotor.getVelocity()) < 5);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT));
  }

  public void resetTestIndicators() {
    m_kickerMotor.resetTestResult();
    m_shooterLeftMotor.resetTestResult();
    m_shooterRightMotor.resetTestResult();
  }
}

package frc.robot.subsystems.shooter;

import static frc.robot.util.EpsilonEquals.epsilonEquals;

import java.util.function.DoubleSupplier;

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
import org.wpilib.command2.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.BusConstants;
import frc.robot.util.Aiming;
import frc.robot.util.PolynomialInterpolation;
import frc.robot.util.Testable;
import frc.robot.util.logging.LoggableSparkFlex;
import frc.robot.util.logging.LoggableSparkFlex.EncoderType;
import frc.robot.util.logging.LoggableSparkFlex.LoggedValue;
import frc.robot.util.logging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import org.wpilib.command2.sysid.SysIdRoutine;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

public class Shooter extends SubsystemBase implements Testable {

  private final TelemetryTable m_shooterTelemetry = Telemetry.getTable("Shooter");

  /* Spark Flex Motors */
  private final LoggableSparkFlex m_shooterLeftMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.SHOOTER_LEFT_MOTOR_ID,
          MotorType.kBrushless,
          m_shooterTelemetry,
          "Left Shooter",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY,
          LoggedValue.CURRENT,
          LoggedValue.OUTPUT_VOLTAGE,
          LoggedValue.APPLIED_OUTPUT);
  private final LoggableSparkFlex m_shooterRightMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.SHOOTER_RIGHT_MOTOR_ID,
          MotorType.kBrushless,
          m_shooterTelemetry,
          "Right Shooter",
          EncoderType.RELATIVE);
  private final LoggableSparkFlex m_kickerMotor =
      new LoggableSparkFlex(
          BusConstants.SHOOTER_BUS,
          ShooterConstants.KICKER_MOTOR_ID,
          MotorType.kBrushless,
          m_shooterTelemetry,
          "Kicker",
          EncoderType.RELATIVE,
          LoggedValue.VELOCITY);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_P_SHOOTER);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", ShooterConstants.K_I_SHOOTER);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", ShooterConstants.K_D_SHOOTER);

  private double m_currentDistance = 0;

  private double m_shooterTrim = -150;
  private boolean m_tagIsSeen = false;

  SparkFlexConfig shooterLeftMotorConfig;

  private PolynomialInterpolation polynomial =
      new PolynomialInterpolation(ShooterConstants.DISTANCES, ShooterConstants.RPMS);

  /** The supplier for the distance to the target
   *  This avoids the Shooter subsystem having to know about the FieldVision subsystem, which would create a circular dependency.
   */
  private final DoubleSupplier m_distanceSupplier;

  private final SysIdRoutine m_shooterSysId = new SysIdRoutine(
    new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null
    ),
    new SysIdRoutine.Mechanism(
        // Direct voltage control; bypasses the normal velocity reference.
        voltage -> m_shooterLeftMotor.setVoltage(voltage.in(Volts)),

        // Log the motor's voltage, position, and velocity.
        log -> log.motor("shooter")
            .voltage(Volts.of(
                m_shooterLeftMotor.getAppliedOutput().get()
                    * m_shooterLeftMotor.getBusVoltage().get()))
            .angularPosition(Rotations.of(
                m_shooterLeftMotor.getEncoder().getPosition().get()))
            .angularVelocity(RotationsPerSecond.of(
                m_shooterLeftMotor.getEncoder().getVelocity().get() / 60.0)),

        this
    )
  );

  public Shooter(DoubleSupplier distanceSupplier) {
    m_distanceSupplier = distanceSupplier;  

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
        .follow(ShooterConstants.SHOOTER_LEFT_MOTOR_ID, true);
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

    // Update the current distance to the target using the distance supplier
    // rather than reading directly from the Limelight or other camera
    // This returns NaN if the target is not seen, which is handled in the next step.
    double distance = m_distanceSupplier.getAsDouble();

    m_tagIsSeen = Double.isFinite(distance);

    if (m_tagIsSeen) {
        m_currentDistance = distance;
    }

    m_shooterTelemetry.log("Distance From Robot to AprilTag", m_currentDistance);
    m_shooterTelemetry.log("AprilTag is recognized", m_tagIsSeen);
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
    m_kickerMotor.setThrottle(ShooterConstants.KICKER_POWER);
  }

  public void shootBall(double rpm) {
    rampUpShooter(rpm);
    m_kickerMotor.setThrottle(ShooterConstants.KICKER_POWER);
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

  public double getYawRotationalRate(double txDegrees) {
    double txAdjusment = Aiming.getYawTxAdjustment(txDegrees);
    m_shooterTelemetry.log("TX Adjustment", txAdjusment);
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

  // Shooter SYSID
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_shooterSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_shooterSysId.dynamic(direction);
  }
}

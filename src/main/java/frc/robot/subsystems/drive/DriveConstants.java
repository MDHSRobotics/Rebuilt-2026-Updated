package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

//SYSTEMCORE import com.pathplanner.lib.config.ModuleConfig;
//SYSTEMCORE import com.pathplanner.lib.config.PIDConstants;
//SYSTEMCORE import com.pathplanner.lib.config.RobotConfig;
//SYSTEMCORE import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Per;

/**
 * Contains constants for the swerve drive that haven't been specified in {@link
 * frc.robot.subsystems.drive.TunerConstants TunerConstants}.
 *
 * <p>Some constants are pre-converted to avoid repeated unit conversions in the main robot loop.
 */
public class DriveConstants {
  private DriveConstants() {}

  /** Distance between center to front bumper */
  public static final Distance CENTER_TO_FRONT_BUMPER_LENGTH = Inches.of(0);

  public static final Distance BUMPER_TO_BUMPER_X_DISTANCE = Inches.of(0);
  public static final Distance BUMPER_TO_BUMPER_Y_DISTANCE = Inches.of(0);

  /** Distance between front left module (cancoder) and front right module (cancoder) */
  private static final Distance FRONT_LEFT_AND_FRONT_RIGHT_CANCODER_DISTANCE =
      TunerConstants.kFrontLeftYPos.minus(TunerConstants.kFrontRightYPos);

  /** Distance between front left module (cancoder) and back left module (cancoder) */
  private static final Distance FRONT_LEFT_AND_BACK_LEFT_CANCODER_DISTANCE =
      TunerConstants.kFrontLeftXPos.minus(TunerConstants.kBackLeftXPos);

  /** Distance from center of robot to a module (cancoder) */
  public static final Distance CENTER_OF_ROBOT_TO_CANCODER_DISTANCE =
      Inches.of(
          Math.hypot(
              FRONT_LEFT_AND_FRONT_RIGHT_CANCODER_DISTANCE.div(2.0).in(Inches),
              FRONT_LEFT_AND_BACK_LEFT_CANCODER_DISTANCE.div(2.0).in(Inches)));

  /** Robot Max Linear Velocity in mph */
  public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  /** Robot Max Angular Velocity in rad per sec */
  public static final double MAX_ANGULAR_VELOCITY =
      RotationsPerSecond.of(MAX_LINEAR_SPEED / CENTER_OF_ROBOT_TO_CANCODER_DISTANCE.in(Meters))
          .in(RadiansPerSecond);

  public static final double MAX_TELEOP_ANGULAR_VELOCITY = 8.77; // rad/s

  /**
   * Constraints for the <a
   * href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html">motion
   * profiles</a> used in custom swerve requests.
   */
  public static final TrapezoidProfile.Constraints LINEAR_MOTION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4.0, 2.0);

  /**
   * Goal tolerance for the <a
   * href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">heading
   * PID controller</a> used in custom swerve requests.
   */
  public static final Angle HEADING_TOLERANCE = Degrees.of(2);

  /**
   * Goal tolerance for the <a
   * href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">x
   * and y PID controllers</a> used in custom swerve requests.
   */
  public static final Distance LINEAR_TOLERANCE = Inches.of(0.5);

  /**
   * Multiply wheel rotations by this number to convert to meters. This accounts for the drive gear
   * ratio. Units: meter / rotations
   *
   * @see <a
   *     href="https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters">How
   *     we convert from rotations to meters</a>
   */
  private static final double WHEEL_ROTATIONS_TO_METERS_CONVERSION =
      2.0 * Math.PI * TunerConstants.kWheelRadius.in(Meters) / TunerConstants.kDriveGearRatio;

  /* PathPlanner Configuration
   * We configure PathPlanner here instead of in the GUI so we can use constants from the code.
   */

  /** PID Constants for PathPlanner translation. */
  //SYSTEMCORE public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);

  /** PID Constants for PathPlanner rotation. */
  //SYSTEMCORE public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);

  /** Robot mass with battery and bumpers in kilograms */
  public static final Mass ROBOT_MASS = Kilograms.of(58.513);

  /**
   * Angular acceleration gain from {@link
   * com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation a SysId routine}. This value needs
   * to be tested and changed later
   */
  private static final Per<VoltageUnit, AngularAccelerationUnit> K_A_ANGULAR =
      Volts.per(DegreesPerSecondPerSecond).ofNative(0);

  /**
   * Linear acceleration gain from {@link
   * com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation a SysId routine}. This value
   * needs to be tested and changed later
   */
  private static final Per<VoltageUnit, LinearAccelerationUnit> K_A_LINEAR =
      VoltsPerMeterPerSecondSquared.ofNative(0 / WHEEL_ROTATIONS_TO_METERS_CONVERSION);

  /**
   * The robot's moment of inertia. This how PathPlanner says to calculate it.
   *
   * @see <a
   *     href="https://pathplanner.dev/robot-config.html#calculating-moi-through-sysid-recommended">the
   *     equation for calculating MOI through SysId</a>
   * @see <a href="https://choreo.autos/usage/estimating-moi/">the equation, but with some units
   *     specified</a>
   * @see <a
   *     href="https://www.chiefdelphi.com/t/question-about-calculating-moi-with-sysid/490893">why
   *     we use drivebase radius instead of trackwidth / 2</a>
   */
  //   private static final MomentOfInertia ROBOT_MOI =
  //       KilogramSquareMeters.of(
  //           ROBOT_MASS.in(Kilograms)
  //               * CENTER_OF_ROBOT_TO_CANCODER_DISTANCE.in(Meters)
  //               * (K_A_ANGULAR.in(VoltsPerRadianPerSecondSquared)
  //                   / K_A_LINEAR.in(VoltsPerMeterPerSecondSquared)));
  private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(4.623786);

  /**
   * Wheel coefficient of friction for <a
   * href="https://www.vexrobotics.com/colsonperforma.html">Colson wheels.</a>
   */
  public static final double WHEEL_COF = 1.0;

  /** The swerve module config to be used for every module. */
  private static final ModuleConfig MODULE_CONFIG =
      new ModuleConfig(
          TunerConstants.kWheelRadius,
          TunerConstants.kSpeedAt12Volts,
          WHEEL_COF,
          DCMotor.getFalcon500(1),
          TunerConstants.kDriveGearRatio,
          TunerConstants.kSlipCurrent,
          1);

  /**
   * The locations of the modules relative to the center of the robot. The order is FL, FR, BL, and
   * BR.
   */
  private static final Translation2d[] MODULE_OFFSETS =
      new Translation2d[] {
        new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
        new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
        new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos),
        new Translation2d(TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos)
      };

  public static final RobotConfig PATHPLANNER_CONFIG =
      new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, MODULE_OFFSETS);

  // public static final PathConstraints ON_THE_FLY_CONSTRAINTS = new PathConstraints(
  //         LINEAR_MOTION_CONSTRAINTS.maxVelocity,
  //         LINEAR_MOTION_CONSTRAINTS.maxAcceleration,
  //         Units.degreesToRadians(540),
  //         Units.degreesToRadians(540),
  //         12);
  // public static final PathConstraints CORAL_STATION_CONSTRAINTS =
  //         new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(540),
  // 12);

  /* Swerve Setpoint Generator Constants */
  /**
   * The maximum angular velocity of the steer motor in radians per second.
   *
   * <p>We limit the voltage here to 7 V because the highest observed voltage is currently about 8
   * V.
   *
   * <p>Units: volts / volts per radian per second
   */
  public static final double MAX_STEER_VELOCITY = 7.0 / TunerConstants.steerGains.kV;

  /**
   * This can safely be reused by multiple swerve requests because it has no internal state (as of
   * FRC 2026).
   */
  public static final SwerveSetpointGenerator SWERVE_SETPOINT_GENERATOR =
      new SwerveSetpointGenerator(PATHPLANNER_CONFIG, MAX_ANGULAR_VELOCITY);
}

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.requests.DriveFacingAngle;
import java.util.function.DoubleSupplier;

/** This class provides instanced command factories for swerve drive aiming */
public class AimingCommand {
  private final CommandSwerveDrivetrain m_drivetrain;

  private final DoubleSupplier m_velocityXSupplier;
  private final DoubleSupplier m_velocityYSupplier;
  private final DoubleSupplier m_deadbandSupplier;

  // NetworkTables
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_cameraTable = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

  // Swerve Request
  private final DriveFacingAngle m_driveFacingAngle =
      new DriveFacingAngle(
              DriveConstants.ROTATION_PID.kP,
              DriveConstants.MAX_ANGULAR_VELOCITY,
              DriveConstants.SWERVE_SETPOINT_GENERATOR,
              Constants.UPDATE_PERIOD)
          .withTolerance(DriveConstants.HEADING_TOLERANCE)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * The current target by whichever command is running. You don't need to worry about multiple
   * commands accessing this because only one command can run at a time.
   */
  private Pose2d m_currentTargetPose;

  /** Logs the target pose to NetworkTables. Use this whenever you calculate a new target pose. */
  private final StructPublisher<Pose2d> m_targetPosePub =
      m_inst.getTable("DriveState").getStructTopic("Target Pose", Pose2d.struct).publish();

  /**
   * Gets the ID of the primary in-view apriltag.
   *
   * @see <a
   *     href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight
   *     NetworkTables API</a>
   * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
   */
  private final IntegerSubscriber m_apriltagIDSub =
      m_cameraTable.getIntegerTopic("tid").subscribe(0);

  /**
   * Constructs an object that provides <a
   * href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced
   * command factories</a> for swerve drive aiming.
   *
   * @param drivetrain The drivetrain to drive and aim with.
   * @param velocityXSupplier A method reference or lambda that returns X velocity.
   * @param velocityYSupplierA method reference or lambda that returns Y velocity.
   * @param deadbandSupplier A method reference or lambda that returns deadband.
   * @param leftYSupplier A method reference or lambda that returns a left joystick's Y value.
   * @param leftXSupplier A method reference or lambda that returns a left joystick's X value.
   * @param rightYSupplier A method reference or lambda that returns a right joystick's Y value.
   * @param rightXSupplier A method reference or lambda that returns a right joystick's X value.
   */
  public AimingCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier deadbandSupplier) {
    m_drivetrain = drivetrain;
    m_velocityXSupplier = velocityXSupplier;
    m_velocityYSupplier = velocityYSupplier;
    m_deadbandSupplier = deadbandSupplier;
  }

  public Command alignWithTower() {
    return m_drivetrain.startRun(
        () -> {
          // Must call reset before using this swerve request
          m_driveFacingAngle.resetRequest();
          Alliance alliance = MatchState.getAlliance().orElseThrow();
          if (alliance == Alliance.BLUE) {
            m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[31]);
          } else if (alliance == Alliance.RED) {
            m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[15]);
          }
        },
        () ->
            m_drivetrain.setControl(
                m_driveFacingAngle
                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                    .withDeadband(m_deadbandSupplier.getAsDouble())));
  }

  public Command alignWithHub() {
    return m_drivetrain.startRun(
        () -> {
          // Must call reset before using this swerve request
          m_driveFacingAngle.resetRequest();
          Alliance alliance = MatchState.getAlliance().orElseThrow();
          if (alliance == Alliance.BLUE) {
            m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[25]);
          } else if (alliance == Alliance.RED) {
            m_driveFacingAngle.withTargetDirection(FieldConstants.APRILTAG_ROTATIONS[9]);
          }
        },
        () ->
            m_drivetrain.setControl(
                m_driveFacingAngle
                    .withVelocityX(m_velocityXSupplier.getAsDouble())
                    .withVelocityY(m_velocityYSupplier.getAsDouble())
                    .withDeadband(m_deadbandSupplier.getAsDouble())));
  }
}

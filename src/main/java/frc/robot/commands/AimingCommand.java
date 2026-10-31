package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;

import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
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

  // Swerve Request
  private final DriveFacingAngle m_driveFacingAngle =
      new DriveFacingAngle(
              DriveConstants.ROTATION_PID.kP,
              DriveConstants.MAX_ANGULAR_VELOCITY,
              DriveConstants.SWERVE_SETPOINT_GENERATOR,
              Constants.UPDATE_PERIOD
          )
          .withTolerance(DriveConstants.HEADING_TOLERANCE)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Constructs an object that provides <a
   * href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced
   * command factories</a> for swerve drive aiming.
   *
   * @param drivetrain The drivetrain to drive and aim with.
   * @param velocityXSupplier A method reference or lambda that returns X velocity.
   * @param velocityYSupplierA method reference or lambda that returns Y velocity.
   * @param deadbandSupplier A method reference or lambda that returns deadband.
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

  /**
   * Align with one of the apriltags on the hub, based on alliance color
   * @return
   */
  public Command alignWithHubApriltag() {
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

  /**
   * Align to Hub based on the robot's current pose relative to the center of the hub
   * @return command
   */
  public Command alignWithHubCenter() {
    return m_drivetrain.startRun(
        () -> m_driveFacingAngle.resetRequest(),
        () -> {
          var alliance = MatchState.getAlliance();
          
          // Preserve translation without rotating if alliance is unknown.
          if (alliance.isEmpty()) {
            m_driveFacingAngle.withTargetDirection(
                m_drivetrain.getState().Pose.getRotation());
          } else {
            var hubCenter =
                alliance.get() == Alliance.BLUE
                    ? FieldConstants.BLUE_HUB_CENTER
                    : FieldConstants.RED_HUB_CENTER;

            var robotPose = m_drivetrain.getState().Pose;
            var toHub = hubCenter.minus(robotPose.getTranslation());

            Rotation2d deltaAngle = toHub.getNorm() > 0.01
                    ? toHub.getAngle().get()
                    : robotPose.getRotation();
            
            // Avoid an undefined aiming direction at the target center.
            m_driveFacingAngle.withTargetDirection(deltaAngle);

          }

          m_drivetrain.setControl(
              m_driveFacingAngle
                  .withVelocityX(m_velocityXSupplier.getAsDouble())
                  .withVelocityY(m_velocityYSupplier.getAsDouble())
                  .withDeadband(m_deadbandSupplier.getAsDouble()));
        });
  }
}

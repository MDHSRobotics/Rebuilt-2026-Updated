package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;
import org.wpilib.tunable.Selectable;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperPowers;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Consumer;

/**
 * This util class is specific to our Pathplanner path naming scheme for Rebuilt-2026. It can use
 * the individual paths defined in Pathplanner to dynamically create a composite auto command
 * consisting of paths and actions. This is done using parameters which can be set in the dashboard.
 */
public class DynamicAutoCreator {
  private static final TunableTable m_autoTunables = Tunables.getTable("Auto");

  private final Selectable<String> m_autoType = new Selectable<>();
  private final Selectable<String> m_startingPositionChooser = new Selectable<>();
  private final Selectable<String> m_actionOneChooser = new Selectable<>();

  private final Consumer<Pose2d> m_odometryResetter;
  private final AutoTimer m_autoTimer = new AutoTimer();
  private Command m_dynamicAutoSequence = null;

  // Subsystems
  private final Shooter m_shooter;
  private final Hopper m_hopper;
  public final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DynamicAutoCreator(
      Consumer<Pose2d> odometryResetter,
      Shooter shooter,
      Hopper hopper,
      CommandSwerveDrivetrain drivetrain) {
    m_odometryResetter = odometryResetter;
    m_shooter = shooter;
    m_hopper = hopper;
    m_drivetrain = drivetrain;
  }

  /*
   * This method publishes to the dashboard parameter settings that can be used to dynamically
   * create an auto command. These parameters are things like starting position and
   * shooting strategy.
   */
  public void publishParameters() {

    // Select whether to use a dynamic or static auto command
    m_autoType.add("Dynamic", "Dynamic");
    m_autoType.addDefault("Static", "Static");
    m_autoType.onChange(this::updateDynamicCommand);
    m_autoTunables.publish("Type of Auto Command", m_autoType);

    // Starting position option
    m_startingPositionChooser.addDefault("Top", "Top to ");
    m_startingPositionChooser.add("Middle", "Middle to ");
    m_startingPositionChooser.add("Bottom", "Bottom to ");
    m_startingPositionChooser.onChange(this::updateDynamicCommand);
    m_autoTunables.publish("Starting Position", m_startingPositionChooser);

    // Options for first action
    m_actionOneChooser.addDefault("Shoot", "Shoot ball");
    m_actionOneChooser.onChange(this::updateDynamicCommand);
    m_autoTunables.publish("Action 1", m_actionOneChooser);

    // Try to generate a dynamic auto command based on the initial parameter settings
    updateDynamicCommand("");
  }

  private void updateDynamicCommand(String changedSetting) {
    String autoType = m_autoType.getSelected();
    if (autoType.equals("Static")) {
      // Static chosen so clear dynamic command
      m_dynamicAutoSequence = null;
    } else {
      // Create a dynamic command based on current settings of auto parameters
      createOneShootingSequenceAuto();
    }
  }


  private void createOneShootingSequenceAuto() {
    try {
      String pathName = m_startingPositionChooser.getSelected();
      pathName += m_actionOneChooser.getSelected();
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      m_dynamicAutoSequence =
          Commands.sequence(
              resetOdometryCommand(path.getStartingHolonomicPose().orElseThrow()),
              Commands.waitSeconds(1),
              Commands.deadline(
                  AutoBuilder.followPath(path),
                  Commands.runOnce(m_autoTimer::resetAndStart),
                  Commands.run(() -> m_shooter.rampUpShooter()).withTimeout(0.5),
                  Commands.run(() -> m_shooter.shootBall()),
                  Commands.runOnce(m_autoTimer::stopAndPublish)));
    } catch (Exception e) {
      DriverStationErrors.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
      System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());
      m_dynamicAutoSequence = null;
      return;
    }
  }


  public Command createShootingAutoSequence() {
    Command auto_command =
        new SequentialCommandGroup(
            Commands.run(() -> m_shooter.rampUpShooter(), m_shooter).withTimeout(4),
            new ParallelCommandGroup(
                Commands.run(() -> m_hopper.runHopper(HopperPowers.INTAKE), m_hopper),
                Commands.run(() -> m_shooter.shootBall(), m_shooter).withTimeout(6)));
    return auto_command;
  }

  public Command createMiddleShootingAutoSequence() {
    final var idle = new SwerveRequest.Idle();
    Command auto_command =
        new SequentialCommandGroup(
            m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.ZERO)),
            m_drivetrain
                .applyRequest(
                    () -> m_drive.withVelocityX(-1).withVelocityY(0).withRotationalRate(0))
                .withTimeout(1.5),
            m_drivetrain.applyRequest(() -> idle).withTimeout(0.5),
            createShootingAutoSequence());
    return auto_command;
  }

  public Command createMiddleShootingRampAutoSequence() {
    final var idle = new SwerveRequest.Idle();
    Command auto_command =
        new SequentialCommandGroup(
            m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.ZERO)),
            m_drivetrain
                .applyRequest(
                    () -> m_drive.withVelocityX(-1).withVelocityY(0).withRotationalRate(0))
                .withTimeout(1),
            m_drivetrain.applyRequest(() -> idle).withTimeout(0.5),
            // createShootingAutoSequence(),
            m_drivetrain
                .applyRequest(() -> m_drive.withVelocityX(0).withVelocityY(1).withRotationalRate(0))
                .withTimeout(2.5));

    return auto_command;
  }

  public Command resetOdometryCommand(Pose2d startingPose) {
    return Commands.runOnce(
        () -> {
          Pose2d newStartingPose = startingPose;
          if (MatchState.getAlliance().orElseThrow() == Alliance.RED) {
            newStartingPose = FlippingUtil.flipFieldPose(startingPose);
          }
          m_odometryResetter.accept(newStartingPose);
        });
  }


  /* This method returns the dynamicly-generated auto command based on
   * options set in the dashboard. If no settings have been selected, return null.
   */
  public Command getCommand() {
    return m_dynamicAutoSequence;
  }
}

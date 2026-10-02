// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandDualShock4Controller;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AimingCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperPowers;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.DynamicAutoCreator;
import frc.robot.util.HubStatus;
import frc.robot.util.Testable;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  // Robot Speed from 0% to 100%
  private double m_robotSpeed = 1.0;

  private double m_testShooterRPM = 2500;
  private boolean m_isLocked = false;

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(getDeadband())
          .withRotationalDeadband(getRotationalDeadband())
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake m_brake =
      new SwerveRequest.SwerveDriveBrake()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.Position);

  private final AimingCommand m_AimingCommand =
      new AimingCommand(
          m_drivetrain, () -> getVelocityX(), () -> getVelocityY(), () -> getDeadband());

  // Autonomous Chooser - A set of options for specifying the active autonomous command from a
  // dashboard like Elastic
  private Selectable<Command> m_staticAutoChooser;

  /* Autonomous Creator - This dynamically creates commands based on settings in the Elastic Auto tab */
  private final DynamicAutoCreator m_dynamicAutoCreator =
      new DynamicAutoCreator(this::resetFieldPosition, m_shooter, m_hopper, m_drivetrain);

  private final DriveTelemetry m_logger = new DriveTelemetry(DriveConstants.MAX_LINEAR_SPEED);

  /* Controllers  */
private final CommandDualShock4Controller m_driverController =
    new CommandDualShock4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
private final CommandGamepad m_operatorController =
    new CommandGamepad(ControllerConstants.OPERATOR_CONTROLLER_PORT);


  // Limiters for smoother controller input
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  private final List<Testable> testableSubsystems = List.of(m_intake, m_hopper, m_shooter);

private Trigger m_autoAlignCanceled =
    new Trigger(() -> Math.abs(m_driverController.getRightX()) > 0.1);
private Trigger m_shoot =
    new Trigger(() -> m_driverController.getR2() > 0.5);
private Trigger m_intakes =
    new Trigger(() -> m_driverController.getL2() > 0.5);


  // Power Distribution Hub
  // public EnergyMonitor energyMonitor = new EnergyMonitor();

  public RobotContainer() {
    setDefaultCommands();
    configureDriverControllers();
    configureOperatorControllers();
    registerNamedCommands();
    setupAutoCommandOptions();
    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return m_drivetrain;
  } 

  /* Define the possible auto command options that can be chosen from the dashboard.
   * This includes:
   *  - Pre-defined auto commands from PathPlanner
   *  - Explicitly defined auto commands
   *  - Dynamically generated commands using parameter settings displayed on the dashboard
   *    (such as starting position, paths, and actions)
   */
    private void setupAutoCommandOptions() {

     // PathPlanner's buildAutoChooser() returns the old SendableChooser type,
     // so build the chooser ourselves instead.
     m_staticAutoChooser = new Selectable<>();
     m_staticAutoChooser.addDefault("None", Commands.none());

        // Explicitly add any other auto commands
     m_staticAutoChooser.add("------------------------", Commands.none());
     m_staticAutoChooser.add("Print Test", new RunCommand(() -> System.out.println("Test")));
     m_staticAutoChooser.add(
        "Shooting only", m_dynamicAutoCreator.createShootingAutoSequence());
     m_staticAutoChooser.add(
        "Middle Shooting", m_dynamicAutoCreator.createMiddleShootingAutoSequence());
     m_staticAutoChooser.add(
        "Middle Shooting and to ramp", m_dynamicAutoCreator.createMiddleShootingRampAutoSequence());

     // Publish the auto command chooser to the dashboard
     Tunables.publish("Static auto commands", m_staticAutoChooser);

      // Publish any dynamic auto parameters to the dashboard
      m_dynamicAutoCreator.publishParameters();

     // Commands can be published directly, so they still show as buttons on the dashboard
     Tunables.publish("Smoke Test", buildFullTestSequence());
     Tunables.publish("Intake Smoke Test", buildSubsystemTestSequence(0));
     Tunables.publish("Hopper Smoke Test", buildSubsystemTestSequence(1));
     Tunables.publish("Shooter Smoke Test", buildSubsystemTestSequence(2));
    }

  // Named Commands for Autonomous
  private void registerNamedCommands() {
    /* 
    NamedCommands.registerCommand(
        "Ramp Up Shooter", Commands.run(() -> m_shooter.rampUpShooter(), m_shooter).withTimeout(2));
    NamedCommands.registerCommand(
        "Shoot Balls",
        new ParallelCommandGroup(
            Commands.run(() -> m_shooter.shootBall(), m_shooter).withTimeout(6),
            Commands.run(() -> m_hopper.runHopper(HopperPowers.SHOOT), m_hopper).withTimeout(6)));
    NamedCommands.registerCommand(
        "Deploy Intake", Commands.run(() -> m_intake.runMotors(0.5, 0.5), m_intake).withTimeout(1));
    NamedCommands.registerCommand(
        "Intake Balls",
        Commands.run(() -> m_intake.runSpinner(IntakeConstants.INTAKE_SPINNERS_POWER), m_intake)
            .withTimeout(4));
    NamedCommands.registerCommand(
        "Lock on to Hub",
        m_drivetrain
            .applyRequest(
                () ->
                    m_drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(
                            m_shooter.getYawRotationalRate()
                                * DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY))
            .withTimeout(2));
        */
  }

  private void setDefaultCommands() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(getVelocityX())
                    .withVelocityY(getVelocityY())
                    .withRotationalRate(getRotationalRate())
                    .withRotationalDeadband(getRotationalDeadband())));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Subsystem Defaults
    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopMotors(), m_shooter));
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.stopMotors(), m_intake));
    m_hopper.setDefaultCommand(new RunCommand(() -> m_hopper.stopMotors(), m_hopper));
  }

  /**
   * Use this method to map driver controls and commands please use <a href="
   * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&rightTrigger=Quarter+Speed&leftStick=Drive&rightStick=Rotate&aButton=Brake&bButton=&yButton=Lock+in+to+hub&rightBumper=Shoot+Balls&startButton=Reset+Field+Orientation
   * ">this controller map</a> to update and view the current controls.
   */
  private void configureDriverControllers() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController
    //     .povUp()
    //     .whileTrue(
    //         m_drivetrain.sysIdDynamic(
    //             org.wpilib.command2.sysid.SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .povDown()
    //     .whileTrue(
    //         m_drivetrain.sysIdDynamic(
    //             org.wpilib.command2.sysid.SysIdRoutine.Direction.kReverse));
    // m_driverController
    //     .povRight()
    //     .whileTrue(
    //         m_drivetrain.sysIdQuasistatic(
    //             org.wpilib.command2.sysid.SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .povLeft()
    //     .whileTrue(
    //         m_drivetrain.sysIdQuasistatic(
    //             org.wpilib.command2.sysid.SysIdRoutine.Direction.kReverse));

    // Half Speed
    m_driverController.R1().onTrue(Commands.runOnce(() -> m_robotSpeed = 0.5));

    m_driverController.R1().onFalse(Commands.runOnce(() -> m_robotSpeed = 1.0));

    m_driverController.cross().whileTrue(
        m_drivetrain
            .applyRequest(() -> m_brake)
            .andThen(Commands.runOnce(() -> System.out.println("Locking Wheels"))));

    // Reset the field-centric heading on option press.
    m_driverController.options().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_driverController
        .circle()
        .whileTrue(
            new SequentialCommandGroup(
                Commands.run(() -> m_shooter.rampUpShooter(ShooterConstants.RPMS[2]), m_shooter)
                    .withTimeout(2),
                new ParallelCommandGroup(
                    Commands.run(() -> m_shooter.shootBall(ShooterConstants.RPMS[2]), m_shooter),
                    Commands.run(() -> m_hopper.runHopper(HopperPowers.SHOOT), m_hopper))));

    // Shoot Ball
    m_shoot.whileTrue(
            new SequentialCommandGroup(
                Commands.run(() -> m_shooter.rampUpShooter(), m_shooter).withTimeout(2),
                new ParallelCommandGroup(
                    Commands.run(() -> m_shooter.shootBall(), m_shooter),
                    Commands.run(() -> m_hopper.runHopper(HopperPowers.SHOOT), m_hopper))));
    // Set rumble on the driver conroller when the robot is shooting the balls
    m_shoot
        .and(new Trigger(() -> !HubStatus.isHubActive(3, 3)))
        .whileTrue(
            Commands.runEnd(
                () -> {
                    m_driverController.getHID().setRumble(
                        GenericHID.RumbleType.LEFT_RUMBLE, 1.0);
                    m_driverController.getHID().setRumble(
                        GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                },
                () -> {
                    m_driverController.getHID().setRumble(
                        GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
                    m_driverController.getHID().setRumble(
                        GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                }));

    // Spin Intake
    m_intakes
        .whileTrue(
            new ParallelCommandGroup(
                Commands.run(
                    () -> m_intake.runSpinner(IntakeConstants.INTAKE_SPINNERS_POWER),
                    m_intake),
                Commands.run(
                    () -> m_hopper.runHopper(HopperPowers.INTAKE),
                    m_hopper)));

    // Spin Intake Reverse
    m_driverController.L1()
        .whileTrue(
            new ParallelCommandGroup(
                Commands.run(
                    () -> m_intake.runSpinner(-0.9),
                    m_intake),
                Commands.run(
                    () -> m_hopper.runHopper(HopperPowers.INTAKE_REVERSE),
                    m_hopper)));


    // Lock on to hub
    m_driverController
        .triangle()
        .toggleOnTrue(
            m_drivetrain
                .applyRequest(
                    () ->
                        m_drive
                            .withVelocityX(getVelocityX())
                            .withVelocityY(getVelocityY())
                            .withRotationalRate(
                                m_shooter.getYawRotationalRate()
                                    * DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY))
                .until(m_autoAlignCanceled));
    m_driverController.triangle().onTrue(Commands.runOnce(() -> m_isLocked = !m_isLocked));
    m_autoAlignCanceled.onTrue(Commands.runOnce(() -> m_isLocked = false));

    // Face the Hub
    m_driverController.square().whileTrue(m_AimingCommand.alignWithHub());
  }

  /**
   * Use this method to map operator controller controls and commands please use <a href="
   * https://www.padcrafter.com/?templates=Driver+Controller&plat=0&rightTrigger=Spin+Hopper&leftStick=Drive&rightStick=&aButton=&bButton=&dpadDown=Lower+intake&rightBumper=Spin+Intake&leftBumper=Spin+Intake+Reverse&leftTrigger=Spin+Hopper+Reverse&dpadRight=Increase+Shooter+RPM&dpadLeft=Decrease+Shooter+RPM
   * ">this controller map</a> to update and view the current controls.
   */
  private void configureOperatorControllers() {

    /* Intake Commands */

   m_operatorController
    .getHID()
    .pov(POVDirection.DOWN)
    .onTrue(
        Commands.run(
            () -> m_intake.runMotors(0.5, 0.5),
            m_intake)
            .withTimeout(1.5));
    // Deploy and Stow Intake
    // m_operatorController
    //     .leftBumper()
    //     .toggleOnTrue(
    //         new RunCommand(() -> m_intake.deployedPosition(), m_intake)
    //             .andThen(() -> m_intake.stowedPosition()));

    // m_operatorController
    //     .leftBumper()
    //     .and(() -> (m_intake.isDeployed()))
    //     .onTrue(Commands.runOnce(() -> m_intake.stowedPosition(), m_intake));
    // m_operatorController
    //     .leftBumper()
    //     .and(() -> (!m_intake.isDeployed()))
    //     .onTrue(Commands.runOnce(() -> m_intake.deployedPosition(), m_intake));

    // m_operatorController.leftBumper().onTrue(Commands.run(() -> m_intake.runMotors(0.8),
    // m_intake));
    // m_operatorController.b().onTrue(Commands.run(() -> m_intake.runMotors(0.8), m_intake));

    m_operatorController
        .faceUp()
        .whileTrue(
            new ParallelCommandGroup(
                Commands.run(
                    () -> m_shooter.shootBall(m_testShooterRPM),
                    m_shooter),
                Commands.run(
                    () -> m_hopper.runHopper(HopperPowers.SHOOT),
                    m_hopper)));

    m_operatorController
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () -> m_hopper.runHopper(HopperPowers.INTAKE_REVERSE),
                m_hopper));

    m_operatorController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> m_hopper.runHopper(HopperPowers.INTAKE),
                m_hopper));

    // Change Shooter Trim
    m_operatorController
        .dpadRight()
        .onTrue(
            new InstantCommand(
                () -> m_shooter.changeTrim(100)));

    m_operatorController
        .dpadLeft()
        .onTrue(
            new InstantCommand(
                () -> m_shooter.changeTrim(-100)));

  }

  public Command getAutonomousCommand() {
    // First see if a dynamic auto command has been defined
    Command auto_command = m_dynamicAutoCreator.getCommand();
    if (auto_command == null) {

      // If not, get the static auto command selected in the AutoChooser drop-down in the dashboard
      auto_command = m_staticAutoChooser.getSelected();
    }
    if (auto_command == null) {
      System.out.println("Autonomous Command is null");
    }
    return auto_command;
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The linear deadband in meters per second.
   */
  public double getDeadband() {
    return DriveConstants.MAX_LINEAR_SPEED * 0.1 * m_robotSpeed;
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The rotational deadband in radians per second.
   */
  public double getRotationalDeadband() {
    return DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY * 0.15 * m_robotSpeed;
  }

  public double getVelocityX() {
    double input = -m_driverController.getLeftY();
    double limited = m_xLimiter.calculate(input);
    return limited * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
  }

  public double getVelocityY() {
    double input = -m_driverController.getLeftX();
    double limited = m_yLimiter.calculate(input);
    return limited * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
  }

 public double getRotationalRate() {
  double input = -m_driverController.getRightX();

  double limited = m_rotLimiter.calculate(input);

  return limited
      * DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY
      * m_robotSpeed;
}


  public void resetFieldPosition(Pose2d position) {
    m_drivetrain.resetPose(position);
  }

  public void resetRobotRotation(Rotation2d rotation) {
    m_drivetrain.resetRotation(rotation);
  }

  public void changeTestRpm(double val) {
    m_testShooterRPM += val;
  }

  /** Update dashboard outputs. */
public void updateDashboardOutputs() {
    Telemetry.log("Hub Active", HubStatus.isHubActive());
    Telemetry.log("Locked on to Hub", m_isLocked);
    Telemetry.log("Time to Next Shift", HubStatus.timeToNextShift());
    // energyMonitor.update();
}

  public Command buildFullTestSequence() {
    List<Command> steps = new ArrayList<>();

    // Reset all indicators first
    steps.add(
        Commands.runOnce(
            () -> {
              for (Testable t : testableSubsystems) {
                t.resetTestIndicators();
              }
            }));

    for (Testable t : testableSubsystems) {
      steps.add(t.test());
      steps.add(new WaitCommand(1.0));
    }

    return new SequentialCommandGroup(steps.toArray(new Command[0]));
  }

  public Command buildSubsystemTestSequence(int index) {
    List<Command> steps = new ArrayList<>();

    // Reset all indicators first
    steps.add(
        Commands.runOnce(
            () -> {
              for (Testable t : testableSubsystems) {
                t.resetTestIndicators();
              }
            }));

    steps.add(testableSubsystems.get(index).test());

    return new SequentialCommandGroup(steps.toArray(new Command[0]));
  }
}

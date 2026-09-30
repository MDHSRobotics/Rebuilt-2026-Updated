// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.SignalLogger;
//import com.pathplanner.lib.commands.FollowPathCommand;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.net.WebServer;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.MatchState;
//import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
//import org.wpilib.driverstation.MatchType;
//import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Tracer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.Constants.VisionConstants;
//SYSTEMCOREimport frc.robot.util.Elastic;
//SYSTEMCORE import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.logging.LoggableSparkFlex;
//import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private DoublePublisher m_matchTimePub;
  private boolean m_hasAppliedRobotRotation;

  private final Tracer m_tracer = new Tracer();

  /* log and replay timestamp and joystick data */
  // private final HootAutoReplay m_timeAndJoystickReplay =
  //     new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {

    m_matchTimePub = NetworkTableInstance.getDefault().getDoubleTopic("Match Time").publish();

    // Configure Limelight Positions
    LimelightHelpers.setCameraPose_RobotSpace(
        VisionConstants.FRONT_LIMELIGHT_NAME,
        VisionConstants.FRONT_LIMELIGHT_FORWARD_DISTANCE,
        0,
        VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE,
        0,
        VisionConstants.FRONT_LIMELIGHT_PITCH,
        0);
    // LimelightHelpers.setCameraPose_RobotSpace(
    //     VisionConstants.BACK_LIMELIGHT_NAME,
    //     VisionConstants.BACK_LIMELIGHT_FORWARD_DISTANCE,
    //     0,
    //     VisionConstants.BACK_LIMELIGHT_UP_DISTANCE,
    //     0,
    //     0,
    //     VisionConstants.BACK_LIMELIGHT_YAW);
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 1);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 1);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 200);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 200);

    SignalLogger.setPath("/media/sda1/logs/");
    SignalLogger.start();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    //URCL.start();
    //FollowPathCommand.warmupCommand().schedule();
    m_hasAppliedRobotRotation = false;

    // Create the webserver for accessing Elastic's saved layout across computers
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    // Initially open the Autonomous tab in Elastic; it will be swapped to Teleop later
    //SYSTEMCORE Elastic.selectTab("Autonomous");

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // m_timeAndJoystickReplay.update();
    m_tracer.clearEpochs();
    CommandScheduler.getInstance().run();
    m_tracer.addEpoch("Command Scheduling");
    m_matchTimePub.set(MatchState.getMatchTime());
    m_tracer.addEpoch("Match Time Logging");
    LoggableSparkFlex.updateAll();
    m_tracer.addEpoch("Spark Flex Logging");
    m_robotContainer.updateDashboardOutputs();
    m_tracer.addEpoch("Smart Dashboard");
    m_tracer.printEpochs();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (!m_hasAppliedRobotRotation) {
      Alliance alliance = MatchState.getAlliance().orElse(null);
      if (alliance == Alliance.BLUE) {
        m_robotContainer.resetRobotRotation(Rotation2d.k180deg);
        m_hasAppliedRobotRotation = true;
      } else if (alliance == Alliance.RED) {
        m_robotContainer.resetRobotRotation(Rotation2d.ZERO);
        m_hasAppliedRobotRotation = true;
      }
    }

    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 1);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      String autoCommandName = m_autonomousCommand.getName();
      System.out.println("Starting Auto: " + autoCommandName);
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 4);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 200);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 4);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
    //SYSTEMCORE Elastic.selectTab("Teleoperated");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 4);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
    m_robotContainer.resetFieldPosition(
        new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.fromDegrees(180)));
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {
    SignalLogger.stop();
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 1);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 1);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 200);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 200);
  }

  @Override
  public void simulationPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Meters;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.net.WebServer;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.MatchState;
//import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
//import org.wpilib.driverstation.MatchType;
//import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.system.Tracer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.Constants.VisionConstants;
//SYSTEMCORE import frc.robot.util.Elastic;
import com.limelightvision.Limelight;
import com.revrobotics.util.StatusLogger;
import com.limelightvision.IMUMode;
import frc.robot.util.logging.LoggableSparkFlex;
//SYSTEMCORE import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private DoublePublisher m_matchTimePub;
  private boolean m_hasAppliedRobotRotation;

  private Limelight m_frontLimelight;

  private final Tracer m_tracer = new Tracer();

  /* log and replay timestamp and joystick data */
  // private final HootAutoReplay m_timeAndJoystickReplay =
  //     new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {

    if (RobotBase.isSimulation()) {
        DriverStationSim.setAllianceStationId(Constants.SIMULATION_ALLIANCE_STATION_ID);
        DriverStationSim.notifyNewData();
    }

    m_matchTimePub = NetworkTableInstance.getDefault().getDoubleTopic("Match Time").publish();

    m_frontLimelight = new Limelight(VisionConstants.FRONT_LIMELIGHT_NAME);
    // Use shared robot orientation for MegaTag2
    m_frontLimelight.setUseSharedOrientation(true);

    // Configure Limelight Positions
    m_frontLimelight.setCameraPose_RobotSpaceOverride(
        VisionConstants.FRONT_LIMELIGHT_FORWARD_DISTANCE,
        0.,
        VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE,
        0.,
        VisionConstants.FRONT_LIMELIGHT_PITCH,
        0.,
        true);

    //SYSTEMCORE: The previous setimumode call passed in a value of 1 (external IMU, seed internal IMU?)
    m_frontLimelight.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);

    m_frontLimelight.setThrottle(200);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 200);

    //  LOGGING SETUP
    StatusLogger.disableAutoLogging();

    if (RobotBase.isSimulation()) { 
        // Simulation: logs under the project directory
        try {
            Files.createDirectories(Path.of("logs/ctre"));
            Files.createDirectories(Path.of("logs/wpilib"));
        } catch (IOException e) {
            e.printStackTrace();
  }
        SignalLogger.setPath("logs/ctre/");
        SignalLogger.start();

        DataLogManager.start("logs/wpilib/");
    } else {
        // SystemCore: logs on USB drive
        SignalLogger.setPath("/u/ctre-logs/");
        SignalLogger.start();

        DataLogManager.start("/u");
    }

    // Capture Driver Station data
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Capture console output
    DataLogManager.logConsoleOutput(true);

    //URCL.start();
    //FollowPathCommand.warmupCommand().schedule();
    m_hasAppliedRobotRotation = false;

    // Create the webserver for accessing Elastic's saved layout across computers
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    // Initially open the Autonomous tab in Elastic; it will be swapped to Teleop later
    //SYSTEMCORE Elastic.selectTab("Autonomous");

    //SYSTEMCORE: This is to make sure that we are passing in the correct time epoch into AddVisionMeasurement in the CommandSwerveDrivetrain subsystem.  
    System.out.println("NT time = " + NetworkTablesJNI.now() / 1_000_000_000.0);
    System.out.println("CTRE time = " + Utils.getCurrentTimeSeconds());

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

    // Feed the robot orientation into the MegaTag2 system for vision processing
    Limelight.setSharedRobotOrientation(
        m_robotContainer
            .getDrivetrain()
            .getState()
            .Pose
            .getRotation()
            .getDegrees());
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
    //SYSTEMCORE: The previous setimumode call passed in a value of 1 (external IMU, seed internal IMU)
    m_frontLimelight.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
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

    //SYSTEMCORE: The previous setimumode call passed in a value of 4 (internal IMU assisted by external IMU?)
    m_frontLimelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);

    m_frontLimelight.setThrottle(0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_frontLimelight.setThrottle(200);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    m_frontLimelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
    m_frontLimelight.setThrottle(0);

    //SYSTEMCORE Elastic.selectTab("Teleoperated");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();


    m_frontLimelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
    m_frontLimelight.setThrottle(0);

    m_robotContainer.resetFieldPosition(
        new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.fromDegrees(180)));
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {
    SignalLogger.stop();

    m_frontLimelight.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
    m_frontLimelight.setThrottle(200);

  }

  @Override
  public void simulationPeriodic() {}
}

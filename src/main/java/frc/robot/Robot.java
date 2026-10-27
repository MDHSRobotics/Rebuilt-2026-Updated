// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

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
import org.wpilib.driverstation.RobotState;
//import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
//import org.wpilib.driverstation.MatchType;
//import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Tracer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
//SYSTEMCORE import frc.robot.util.Elastic;
import com.revrobotics.util.StatusLogger;

import frc.robot.Constants.Mode;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.vision.FieldVisionConstants;
import frc.robot.util.logging.EpochLogger;
import frc.robot.util.logging.LoggableSparkFlex;
//SYSTEMCORE import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private DoublePublisher m_matchTimePub;
  private boolean m_hasAppliedRobotRotation;

  private final Tracer m_tracer = new Tracer();
  private final EpochLogger epochLogger = new EpochLogger(m_tracer);

  /* log and replay timestamp and joystick data */
  // private final HootAutoReplay m_timeAndJoystickReplay =
  //     new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    if (Constants.currentMode == Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(SimulationConstants.SIMULATION_ALLIANCE_STATION_ID);
      DriverStationSim.notifyNewData();
    }

    m_matchTimePub = NetworkTableInstance.getDefault().getDoubleTopic("Match Time").publish();

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
        SignalLogger.setPath("/U/ctre-logs/");
        SignalLogger.start();

        DataLogManager.start("/U/WPI-logs/");
    }

    // Capture Driver Station data
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Capture console output
    DataLogManager.logConsoleOutput(true);

    //URCL.start();
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    m_hasAppliedRobotRotation = false;

    // Create the webserver for accessing Elastic's saved layout across computers
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    // Initially open the Autonomous tab in Elastic; it will be swapped to Teleop later
    //SYSTEMCORE Elastic.selectTab("Autonomous");

    //SYSTEMCORE: This is to make sure that we are passing in the correct time epoch into AddVisionMeasurement in the CommandSwerveDrivetrain subsystem.  
    System.out.println("NT time = " + NetworkTablesJNI.now() / 1_000_000_000.0);
    System.out.println("CTRE time = " + Utils.getCurrentTimeSeconds());

    m_robotContainer = new RobotContainer();

    epochLogger.start();
  }

  /** 
   *  This method logs a comparison of the NetworkTables and CTRE clocks
   *  It is used to verify that the clocks are in sync and that the time epoch is correct 
   *  for vision measurements.
   * 
   *  How to interpret the results:
   *    - The offset should be near zero, and the read window should be small (less than 1ms)
   *      Offset consistently within that uncertainty: no measurable clock difference.
   *    - Stable offset clearly outside it: evidence of different epochs.
   *    - Spikes accompanying wider read windows: likely scheduling jitter
   *
   */
  private void logClockComparison() {
    long ntBefore = NetworkTablesJNI.now();
    double ctreSeconds = Utils.getCurrentTimeSeconds();
    long ntAfter = NetworkTablesJNI.now();

    double ntMidpointSeconds =
        ntBefore * 1e-9 + (ntAfter - ntBefore) * 0.5e-9;

    Logger.recordOutput(
        "ClockDiagnostic/CtreMinusNtSeconds",
        ctreSeconds - ntMidpointSeconds);

    Logger.recordOutput(
        "ClockDiagnostic/ReadWindowSeconds",
        (ntAfter - ntBefore) * 1e-9);
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
    epochLogger.update();

    // Log the clock comparison for diagnostic purposes
    // This is only done in disabled mode to avoid adding extra load during active robot operation
    // It's really not very expensive, but we want to be careful about adding extra load during active robot operation
    if (RobotState.isDisabled()) {
      logClockComparison();
    }
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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    //SYSTEMCORE Elastic.selectTab("Teleoperated");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.resetFieldPosition(
        new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.fromDegrees(180)));
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}

  @Override
  public void simulationInit() {

    // Only initialize the startng position in live simulation, not log replay.
    if (Constants.currentMode == Mode.SIM) {

      // Reset the robot's field position to the calculated starting position.
      m_robotContainer.resetFieldPosition(SimulationConstants.getSimulationStartingPosition() );

      // Keep disabledPeriodic() from replacing this starting heading.
      m_hasAppliedRobotRotation = true;
    }
  }

  @Override
  public void simulationPeriodic() {}
}

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import frc.robot.Constants.VisionConstants;

public class DriveTelemetry {
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public DriveTelemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
    SignalLogger.start();

    /* Set up the module state Mechanism2d telemetry */
    for (int i = 0; i < 4; ++i) {
      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();

  private final DoubleArrayPublisher m_megatag2FrontUpdater =
      m_inst
          .getTable(VisionConstants.FRONT_LIMELIGHT_NAME)
          .getDoubleArrayTopic("robot_orientation_set")
          .publish(PubSubOption.periodic(0.004));

  /** Limelight requires this to be an array of size 6 */
  private double[] m_megatag2Orientation = new double[6];

  /* Robot swerve drive state */
  private final NetworkTable m_driveStateTable = m_inst.getTable("DriveState");
  private final StructPublisher<Pose2d> m_drivePosePub =
      m_driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisVelocities> m_driveSpeedsPub =
      m_driveStateTable.getStructTopic("Speeds", ChassisVelocities.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleStates =
      m_driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleTargets =
      m_driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> m_driveModulePositions =
      m_driveStateTable
          .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
          .publish();
  private final DoublePublisher driveTimestamp =
      m_driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher m_driveOdometryFrequencyPub =
      m_driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable table = m_inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
      };

  private final double[] m_poseArray = new double[3];

  private final DoublePublisher m_linearSpeedPub =
      m_driveStateTable.getDoubleTopic("Linear Speed").publish();

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    long timestampMicroseconds = stateTimestampToNTTimestamp(state.Timestamp);

    /*Send the robot orientation to the limelight for megatag2 */
    m_megatag2Orientation[0] = state.Pose.getRotation().getDegrees();
    m_megatag2Orientation[1] = state.Speeds.omega * 180 / Math.PI;
    m_megatag2FrontUpdater.set(m_megatag2Orientation, timestampMicroseconds);
    // Flushing is ESSENTIAL for the limelight to receive accurate yaw and give accurate pose
    // estimates.
    m_inst.flush();

    /* Telemeterize the swerve drive state */
    m_drivePosePub.set(state.Pose, timestampMicroseconds);
    m_driveSpeedsPub.set(state.Speeds, timestampMicroseconds);
    // m_driveModuleStates.set(state.ModuleStates, timestampMicroseconds);
    // m_driveModuleTargets.set(state.ModuleTargets, timestampMicroseconds);
    // m_driveModulePositions.set(state.ModulePositions, timestampMicroseconds);
    m_driveOdometryFrequencyPub.set(1.0 / state.OdometryPeriod, timestampMicroseconds);

    double linearSpeed = Math.hypot(state.Speeds.vx, state.Speeds.vy);
    m_linearSpeedPub.set(linearSpeed, timestampMicroseconds);

    /* Also write to log file */
    SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
    SignalLogger.writeStruct("DriveState/Speeds", ChassisVelocities.struct, state.Speeds);
    SignalLogger.writeStructArray(
        "DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
    SignalLogger.writeStructArray(
        "DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
    SignalLogger.writeStructArray(
        "DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.set("Field2d");

    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    fieldPub.set(m_poseArray);

    /* Telemeterize each module state to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].velocity / (2 * MaxSpeed));
    }
  }

  /**
   * Converts a {@link com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState SwerveDriveState}
   * timestamp to the timebase required for NetworkTables.
   *
   * <p>This is basically the inverse of calling {@link
   * com.ctre.phoenix6.Utils#fpgaToCurrentTime(double) Utils.fpgaToCurrentTime()}.
   *
   * @param stateTimestampSeconds The SwerveDriveState timestamp in seconds
   * @return The equivalent NetworkTables timestamp in microseconds
   */
  public static long stateTimestampToNTTimestamp(double stateTimeStampSeconds) {
    double NTTimeStampSeconds =
        stateTimeStampSeconds + (Timer.getTimestamp() - Utils.getCurrentTimeSeconds());
    return (long) (NTTimeStampSeconds * 1000000.0);
  }
}

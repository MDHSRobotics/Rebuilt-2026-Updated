package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import org.wpilib.networktables.NetworkTablesJNI;
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

    // SignalLogger is initialized and started in Robot()
    //SignalLogger.start();

    /* Set up the module state Mechanism2d telemetry */
    for (int i = 0; i < 4; ++i) {
      Telemetry.log("Module " + i, m_moduleMechanisms[i]);
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

  private final StructArrayPublisher<SwerveModuleVelocity> m_driveModuleVelocities =
      m_driveStateTable
          .getStructArrayTopic("ModuleVelocities", SwerveModuleVelocity.struct)
          .publish();

  private final StructArrayPublisher<SwerveModuleVelocity> m_driveModuleTargets =
      m_driveStateTable
          .getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct)
          .publish();


  private final StructArrayPublisher<SwerveModulePosition> m_driveModulePositions =
      m_driveStateTable
          .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
          .publish();

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

  /** Accept the swerve drive state and publish it to NetworkTables and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    long timeStampNanoseconds = stateTimestampToNTTimestamp(state.Timestamp);

    /*Send the robot orientation to the limelight for megatag2 */
    m_megatag2Orientation[0] = state.Pose.getRotation().getDegrees();
    m_megatag2Orientation[1] = state.Velocity.omega * 180 / Math.PI;
    m_megatag2FrontUpdater.set(m_megatag2Orientation, timeStampNanoseconds);
    // Flushing is ESSENTIAL for the limelight to receive accurate yaw and give accurate pose
    // estimates.
    m_inst.flush();

    /* Telemeterize the swerve drive state */
    m_drivePosePub.set(state.Pose, timeStampNanoseconds);
    m_driveSpeedsPub.set(state.Velocity, timeStampNanoseconds);
    m_driveModuleVelocities.set(state.ModuleVelocities, timeStampNanoseconds);
    m_driveModuleTargets.set(state.ModuleTargets, timeStampNanoseconds);
    m_driveModulePositions.set(state.ModulePositions, timeStampNanoseconds);
    m_driveOdometryFrequencyPub.set(1.0 / state.OdometryPeriod, timeStampNanoseconds);

    double linearSpeed = Math.hypot(state.Velocity.vx, state.Velocity.vy);
    m_linearSpeedPub.set(linearSpeed, timeStampNanoseconds);

    /* Also write to log file */
    SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);

    SignalLogger.writeStruct("DriveState/Speeds", ChassisVelocities.struct, state.Velocity);

    SignalLogger.writeStructArray("DriveState/ModuleVelocities", SwerveModuleVelocity.struct, state.ModuleVelocities);

    SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleVelocity.struct, state.ModuleTargets);

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
      m_moduleSpeeds[i].setAngle(state.ModuleVelocities[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleVelocities[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleVelocities[i].velocity / (2 * MaxSpeed));
    }
  }

    /**
     * Converts a CTRE SwerveDriveState timestamp to the NetworkTables
     * local timestamp timebase.
     *
     * @param stateTimestampSeconds The SwerveDriveState timestamp in seconds,
     *     using the CTRE current-time timebase
     * @return The equivalent NetworkTables local timestamp in nanoseconds
     */
    public static long stateTimestampToNTTimestamp(double stateTimestampSeconds) {
    long ntNowNanoseconds = NetworkTablesJNI.now();
    double ctreNowSeconds = Utils.getCurrentTimeSeconds();

    return ntNowNanoseconds
        - (long) ((ctreNowSeconds - stateTimestampSeconds) * 1_000_000_000.0);
    }
}

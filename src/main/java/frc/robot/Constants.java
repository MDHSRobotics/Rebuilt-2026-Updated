package frc.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N3;

import frc.robot.subsystems.vision.FieldVisionConstants;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.AllianceStationID;

public final class Constants {
  private Constants() {}

  /** The update period of the robot in seconds. */
  public static final double UPDATE_PERIOD = 0.02;

  public static final boolean TUNING_MODE = false;

  /**
   * The following constants define the runtime mode used by AdvantageKit.
   * The mode is always "REAL" when running on a roboRIO/Systemcore. 
   * Change the value of "simMode" to switch between "SIM" (physics sim) and "REPLAY" (log replay from a file).
   */
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ControllerConstants {
    private ControllerConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  
  public static class BusConstants {
    private BusConstants() {}

    public static final CANPort ENERGY_MONITOR_BUS = CANPort.CAN_S0;
    public static final CANPort INTAKE_BUS = CANPort.CAN_S0;
    public static final CANPort HOPPER_BUS = CANPort.CAN_S0;
    public static final CANPort SHOOTER_BUS = CANPort.CAN_S0;
    public static final CANPort DRIVE_BUS = CANPort.CAN_S1;
    
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";

    /** Distance from the center of the robot to the front limelight lens in Inches */
    public static final double FRONT_LIMELIGHT_FORWARD_DISTANCE = Inches.of(22.416).in(Meters);

    /** Distance from the floor to the front limelight lens in Inches */
    public static final double FRONT_LIMELIGHT_UP_DISTANCE = Inches.of(22.625).in(Meters);

    /** Units: Degrees - Up is positive */
    public static final double FRONT_LIMELIGHT_PITCH = 25;

    /* NOTE THAT THE FOLLOWING VALUES ARE ONLY USED BY THE SHOOTER TO COMPUTE MOTOR POWER
       AND THEY ARE NOT USED ELSEWHERE, SUCH AS FOR POSE ESTIMATION.
       ALSO, NOTE THAT THEIR VALUES ARE NOT CORRECT BUT HAVE BEEN USED IN CALIBRATIONS.
       THEY REALLY SHOULD BE REMOVED AND THE SHOOTER RECOLIBRATED.
    */
    public static final double FRONT_LIMELIGHT_UP_DISTANCE_INCHES_USED_BY_SHOOTER = 23.75;
    public static final double LIMELIGHT_MOUNT_ANGLE_USED_BY_SHOOTER = 30;
  }

  public static class FieldConstants {
    private FieldConstants() {}

    public static final Translation3d[] NO_VISIBLE_TAGS = new Translation3d[0];
    public static final double[] NO_TAG_DISTANCES = new double[0];

    // The field contains april tag and other information about the layout of the playing field.
    public static final Field PLAYING_FIELD =
        Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

    /**
     * Rotations of the Apriltags for aligning perpendicular to them.
     *
     * <p>These rotations correspond to the Z-rotation of the tags plus 180 degrees
     *
     * <p>Index into the array with the id number starting from 1.
     */
    public static final Rotation2d[] APRILTAG_ROTATIONS = {
      Rotation2d.ZERO,
      PLAYING_FIELD.getTagPose(1).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(2).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(3).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(4).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(5).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(6).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(7).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(8).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(9).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(10).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(11).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(12).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(13).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(14).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(15).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(16).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(17).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(18).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(19).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(20).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(21).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(22).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(23).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(24).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(25).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(26).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(27).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(28).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(29).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(30).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(31).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      PLAYING_FIELD.getTagPose(32).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
    };

    /**
     * The distance from the floor to the center of the Hub's apriltag. I got these values from
     * searching up the april tag 2D drawing from FRC Rebuilt. Units: Inches
     */
    public static final double DISTANCE_FROM_FLOOR_TO_HUB_TAG = 44.25;
  }

  public static class SimulationConstants {
    private SimulationConstants() {}

    // Default alliance station ID for simulation only
    public static final AllianceStationID SIMULATION_ALLIANCE_STATION_ID = AllianceStationID.BLUE_2;

    // Dimensions of the simulated robot from bumper to bumper
    // Got these values from AdvantageKit large robot (30 inches)
    public static final double ADVANTAGESCOPE_ROBOT_WIDTH = Inches.of(30).in(Meters);
    public static final double ADVANTAGESCOPE_ROBOT_LENGTH = Inches.of(30).in(Meters);
    /**
     * Returns the starting position of the robot on the field for simulation purposes.
     * This is based on the current alliance.
     * The starting position is calculated to be inset from the walls by half the robot's bumper-to-bumper dimensions.
     *
     * @return The starting pose of the robot on the field.
     */
    public static Pose2d getSimulationStartingPosition() {

      double xInsetFromWall = ADVANTAGESCOPE_ROBOT_LENGTH / 2.0;
      double yInsetFromWall = ADVANTAGESCOPE_ROBOT_WIDTH / 2.0;

      double fieldLength = FieldVisionConstants.aprilTagLayout.getFieldLength();
      double fieldWidth = FieldVisionConstants.aprilTagLayout.getFieldWidth();

      boolean isRed;
      switch (SIMULATION_ALLIANCE_STATION_ID) {
        case AllianceStationID.RED_1:
        case AllianceStationID.RED_2:
        case AllianceStationID.RED_3:
          isRed = true;
          break;
        case AllianceStationID.BLUE_1:
        case AllianceStationID.BLUE_2:
        case AllianceStationID.BLUE_3:
          isRed = false;
          break;
        default:
          throw new IllegalArgumentException("Invalid simulation alliance station ID: " + SIMULATION_ALLIANCE_STATION_ID);
      }

      Pose2d startingPose =
          new Pose2d(
              isRed ? fieldLength - xInsetFromWall : xInsetFromWall,
              isRed ? fieldWidth - yInsetFromWall : yInsetFromWall,
              isRed ? Rotation2d.k180deg : Rotation2d.ZERO);

      return startingPose;
    }
  }

}

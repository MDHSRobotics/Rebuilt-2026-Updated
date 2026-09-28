package frc.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.fields.Fields;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.fields.Field;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N3;

public final class Constants {
  private Constants() {}

  /** The update period of the robot in seconds. */
  public static final double UPDATE_PERIOD = 0.02;

  public static final boolean TUNING_MODE = false;

  public static class ControllerConstants {
    private ControllerConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int DRIVER_CONTROLLER_RIGHT_AXIS = 4;
    public static final int DRIVER_CONTROLLER_R2_AXIS = 3;
  }

  public static class CanBusConstants {

    private CanBusConstants() {}

    // CAN bus for subsystems and other electrical components
    // Ports S0-S4 are Systemcore CAN buses.  
    public static final CANPort SHOOTER_CAN_BUS = CANPort.CAN_S0;
    public static final CANPort HOPPER_CAN_BUS = CANPort.CAN_S0; 
    public static final CANPort INTAKE_CAN_BUS = CANPort.CAN_S0;
    public static final CANPort POWER_DISTRIBUTION_CAN_BUS = CANPort.CAN_S0;
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final String FRONT_LIMELIGHT_NAME = "limelight";

    /** Distance from the center of the robot to the front limelight lens in Inches */
    public static final double FRONT_LIMELIGHT_FORWARD_DISTANCE = Inches.of(22.416).in(Meters);

    public static final double FRONT_LIMELIGHT_FORWARD_DISTANCE_INCHES = 22.416;

    /** Distance from the floor to the front limelight lens in Inches */
    public static final double FRONT_LIMELIGHT_UP_DISTANCE = Inches.of(22.625).in(Meters);

    public static final double FRONT_LIMELIGHT_UP_DISTANCE_INCHES = 23.75;

    /*Units: degrees */
    public static final double LIMELIGHT_MOUNT_ANGLE = 30;

    /** Units: Degrees */
    public static final double FRONT_LIMELIGHT_PITCH = 30;

    // These needs to be tested and adjusted later on
    public static final double FRONT_X_STD_DEV = 0.04;
    public static final double FRONT_Y_STD_DEV = 0.0017;
    public static Vector<N3> FRONT_STD_DEVS =
        VecBuilder.fill(FRONT_X_STD_DEV, FRONT_Y_STD_DEV, Double.MAX_VALUE);

    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

    /** Distance from the center of the robot to the back limelight lens in meters */
    public static final double BACK_LIMELIGHT_FORWARD_DISTANCE = Inches.of(0).in(Meters);

    /** Distance from the floor to the front limelight lens in meters */
    public static final double BACK_LIMELIGHT_UP_DISTANCE = Inches.of(0).in(Meters);

    /** Units: degree */
    public static final double BACK_LIMELIGHT_YAW = 180;

    public static final double BACK_X_STD_DEV = 0;
    public static final double BACK_Y_STD_DEV = 0;
    public static Vector<N3> BACK_STD_DEVS =
        VecBuilder.fill(BACK_X_STD_DEV, BACK_Y_STD_DEV, Double.MAX_VALUE);
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
}

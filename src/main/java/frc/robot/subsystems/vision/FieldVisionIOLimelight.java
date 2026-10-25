// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.system.Timer;

import com.ctre.phoenix6.Utils;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.limelightvision.Limelight;

import frc.robot.Constants;

import com.limelightvision.IMUMode;
import org.wpilib.driverstation.RobotState;

/** IO implementation for real Limelight hardware. */
public class FieldVisionIOLimelight implements FieldVisionIO {

  private final String cameraName;
  private final Transform3d robotToCamera; // Position and orientation of camera relative to robot

  private final Limelight limelight;

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleSubscriber heartbeatSubscriber;

  private double lastHeartbeat = Double.NaN;
  private double lastHeartbeatTime = Double.NEGATIVE_INFINITY;

  private boolean lastEnabled = false;
  private boolean configurationInitialized = false;

  private static final double CONNECTION_TIMEOUT = 0.5; // seconds

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public FieldVisionIOLimelight(String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
    this.cameraName = name;
    this.robotToCamera = robotToCamera;

    limelight = new Limelight(name);

    limelight.setCameraPose_RobotSpaceOverride(
        new Pose3d(
            robotToCamera.getTranslation(),
            robotToCamera.getRotation()),
        true);
        
    var table = NetworkTableInstance.getDefault().getTable(cameraName);
    this.rotationSupplier = rotationSupplier;

    // robot_orientation_set layout:
    // [yaw, yawRate, pitch, pitchRate, roll, rollRate]
    // Angles are degrees; angular rates are degrees per second.
    // Currently only yaw is supplied; all other components are zero.
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);  
  }

  // Get a human-friendly name for this interface
  public String getName() {
    return this.cameraName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    boolean enabled = RobotState.isEnabled();

    // Reduce the heat of the Limelight when the robot is disabled,
    // and restore full processing when enabled.
    if (!configurationInitialized || enabled != lastEnabled) {
        if (enabled) {
            // Robot enabled: full camera processing
            limelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
            limelight.setThrottle(0);
        } else {
            // Robot disabled: reduce processing
            limelight.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
            limelight.setThrottle(200);
        }
        lastEnabled = enabled;
        configurationInitialized = true;
    }

    // Update connection status based on whether Limelight has published a recent heartbeat
    double currentTime = Timer.getTimestamp();
    double heartbeat = heartbeatSubscriber.get();

    if (heartbeatSubscriber.getLastChange() != 0 && heartbeat != lastHeartbeat) {
        lastHeartbeat = heartbeat;
        lastHeartbeatTime = currentTime;
    }

    inputs.connected = (currentTime - lastHeartbeatTime) < CONNECTION_TIMEOUT;

    inputs.hasTarget = limelight.hasTarget(); 

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        // NOTE: !!!!
        // The following provides Limelight with the current robot heading but not its angular velocity.
        // It's fine for initial testing. 
        // We can revisit supplying the actual yaw rate if we see problems during rotation.
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Both pose topics use the WPILib blue-alliance field origin:
    //   botpose_wpiblue     = MegaTag1
    //   botpose_orb_wpiblue = MegaTag2
    //
    // Each rawSample is one timestamped NetworkTables update.
    // rawSample.timestamp is local NT time in nanoseconds (WPILib 2027 alpha-7).
    // rawSample.value contains an 11-value header followed by per-tag records:
    //
    // Header:
    //   [0..2]  Robot field position: X, Y, Z (meters)
    //   [3..5]  Robot field orientation: roll, pitch, yaw (degrees)
    //   [6]     Total capture + pipeline latency (milliseconds)
    //   [7]     Number of tags contributing to the pose
    //   [8]     Tag span (meters)
    //   [9]     Average distance from camera to contributing tags (meters)
    //   [10]    Average contributing-tag area (percentage of image)
    //
    // Starting at [11], each tag occupies 7 values:
    //   [+0] ID
    //   [+1] Horizontal offset from principal pixel (degrees)
    //   [+2] Vertical offset from principal pixel (degrees)
    //   [+3] Target area (percentage of image)
    //   [+4] Distance to camera (meters)
    //   [+5] Distance to robot (meters)
    //   [+6] Pose ambiguity
    //
    // readQueue() drains buffered updates, so an empty queue produces no new
    // observations. It avoids repeatedly processing a cached latest value.
    // Only updates retained by the NT subscription can be read.
    
    for (var rawSample : megatag1Subscriber.readQueue()) {

      // Require the complete 11-value header and at least one contributing tag.
      if (rawSample.value.length < 11 || rawSample.value[7] <= 0) continue;

      // Walk complete 7-value tag records after the header.
      // i points to the tag ID; i + 6 is the record's final value (ambiguity).
      // A trailing incomplete record is ignored.
      for (int i = 11; i + 6 < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp converted from local NetworkTables time to CTRE time,
              // corrected for Limelight processing latency
              getVisionTimestamp(rawSample.timestamp, rawSample.value[6]),

              // 3D pose estimate
              parsePose(rawSample.value),

              // First tag's ambiguity is at 11 + 6 = 17.
              // FieldVision checks this only for single-tag observations.
              // If the first tag record is absent, this implementation uses 0.0;
              // that is a fallback, not a measured zero ambiguity.
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,              

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {

      // Require the complete 11-value header and at least one contributing tag.
      if (rawSample.value.length < 11 || rawSample.value[7] <= 0) continue;

      // Walk complete 7-value tag records after the header.
      // i points to the tag ID; i + 6 is the record's final value (ambiguity).
      // A trailing incomplete record is ignored.
      for (int i = 11; i + 6 < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp converted from local NetworkTables time to CTRE time,
              // corrected for Limelight processing latency
              getVisionTimestamp(rawSample.timestamp, rawSample.value[6]),

              // 3D pose estimate
              parsePose(rawSample.value),

              // MegaTag2 uses supplied robot orientation to resolve pose ambiguity.
              // Store 0.0 so the single-tag ambiguity filter does not reject MT2
              // based on the geometric ambiguity metric used for MT1.
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Converts botpose header values [0..5] into a robot Pose3d.
   * Position is already in meters; roll/pitch/yaw are converted from
   * Limelight's degrees to the radians required by Rotation3d.
   * The caller must validate the array length before calling.
   */
  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  /* Gets the timestamp for a vision measurement based on NetworkTables timestamp and latency.
   * This allows FieldVision to continue using a simple double timestampSeconds without knowing anything about CTRE.
   * The conversion occurs at the hardware IO boundary, where it belongs.
   */

  
  private static double getVisionTimestamp(
    long ntTimestampNanoseconds,
    double latencyMilliseconds) {

    // NetworkTables timestamps are nanoseconds in WPILib 2027.
    // *** IMPORTANT: This assumes that the NetworkTables sample timestamp and NetworkTablesJNI.now() 
    // *** share the same clock domain of nanoseconds. This was confirmed by reading the code in 
    // *** NetworkTablesJNI.java and TimestampedDoubleArray.timestamp
    // ***
    double ntNowSeconds = NetworkTablesJNI.now() * 1.0e-9;
    double ctreNowSeconds = Utils.getCurrentTimeSeconds();  
    
    // Measurement time = current CTRE time - sample age - camera latency.
    // Subtract camera latency exactly once to estimate capture time.
    return ctreNowSeconds
        - (ntNowSeconds - ntTimestampNanoseconds * 1.0e-9)
        - latencyMilliseconds * 1.0e-3;
}
}

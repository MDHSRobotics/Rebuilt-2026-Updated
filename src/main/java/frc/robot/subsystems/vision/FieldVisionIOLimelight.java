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
  public FieldVisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    cameraName = name;

    limelight = new Limelight(name);

    limelight.setCameraPose_RobotSpaceOverride(
        Constants.VisionConstants.FRONT_LIMELIGHT_FORWARD_DISTANCE,
        0.0,
        Constants.VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE,
        0.0,
        Constants.VisionConstants.FRONT_LIMELIGHT_PITCH,
        0.0,
        true);
        
    var table = NetworkTableInstance.getDefault().getTable(cameraName);
    this.rotationSupplier = rotationSupplier;
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
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length < 11 || rawSample.value[7] <= 0) continue;
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

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length < 11 || rawSample.value[7] <= 0) continue;
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

              // Ambiguity, zeroed because the pose is already disambiguated
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

    return ctreNowSeconds
        - (ntNowSeconds - ntTimestampNanoseconds * 1.0e-9)
        - latencyMilliseconds * 1.0e-3;
}
}

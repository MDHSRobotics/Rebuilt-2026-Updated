// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;
import frc.robot.Constants;

public class FieldVisionConstants {

  /** Are we running a physics simulator, but with a real camera connected. */
  public static final boolean hardwareInTheLoop = false;

  // Notes about robot to camera transforms
  // 1. These are not used for Limelight; instead you configure it in web UI. However, we
  //    include transforms even for Limelight because they are used by simulation.
  //
  // 2. Pay attention to the signs of the camera rotation because they are not intuitive,
  //    especially the pitch. You would think that a camera pointed down to the floor
  //    would have a negative pitch but actually it should be positive. All the angles use
  //    the right-hand rule: point the thumb on your right hand in the positive direction
  //    of the axis you are going to rotate about. Then your finger curl in the positive
  //    direction of rotation. The convention is that the x-axis is positive toward the
  //    front of the robot, y-axis is positive toward the left of the robot; z-axis is
  //    positive vertically up. So pitch is a rotation about y-axis so point your right
  //    thumb toward the left of the robot and you will see that curling your fingers
  //    will tilt the front of the robot down.

  // The following is a definition of all cameras on the real robot. These are also used
  // by simulation
  public static CameraSpec[] robotCameras = {
    new CameraSpec(
        VisionType.LIMELIGHT,
        Constants.VisionConstants.FRONT_LIMELIGHT_NAME,
        new Transform3d(
            Constants.VisionConstants.FRONT_LIMELIGHT_FORWARD_DISTANCE,
            0.0,
            Constants.VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE,
            new Rotation3d(
                // Don't forget that the pitch should be negative to point up
                0.0, -Math.toRadians(Constants.VisionConstants.FRONT_LIMELIGHT_PITCH), 0.0)))
  };

  // The following is a definition of all cameras on a test harness used to testing
  // vision hardware-in-the-loop in simulation mode.
  public static CameraSpec[] hardwareInLoopCameras = {};

  // AprilTag layout
  public static Field aprilTagLayout =
      Fields.FRC_2026_REBUILT_WELDED.loadField();

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.15;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

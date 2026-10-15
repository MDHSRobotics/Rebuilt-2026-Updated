package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraSpec {

  public VisionType visionType; // Type of vision processor
  public String cameraName; // Unique name for this camera
  public Transform3d robotToCamera; // Tranformation from robot origin to camera

  public CameraSpec(VisionType type, String name, Transform3d robotToCameraXform) {
    this.visionType = type;
    this.cameraName = name;
    this.robotToCamera = robotToCameraXform;
  }
}

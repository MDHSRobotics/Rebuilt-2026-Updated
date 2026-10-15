package frc.robot.subsystems.vision;

public class FieldVisionIOReplayable implements FieldVisionIO {
  private final String cameraName;

  public FieldVisionIOReplayable(String cameraName) {
    this.cameraName = cameraName;
  }

  // Get a human-friendly name for this interface
  public String getName() {
    return cameraName;
  }
}

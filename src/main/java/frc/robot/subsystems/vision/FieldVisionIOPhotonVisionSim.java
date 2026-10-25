// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.FieldVisionConstants.aprilTagLayout;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class FieldVisionIOPhotonVisionSim extends FieldVisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public FieldVisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {

    super(name, robotToCamera);

    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    // Use default properties for now, but we can add a way to configure them later if needed
    // This includes the camera's field of view, resolution, and other properties that affect how it sees the world
    var cameraProperties = new SimCameraProperties();

    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);

    // Disable image rendering to reduce simulation CPU usage.
    // Simulated tag detections and pose estimates still work, even if all of the folowing are fale

    // If true, Renders the simulated camera image without processing overlays.
    // Only useful if you want to view it
    cameraSim.enableRawStream(true);

    // Renders the image with detection overlays.
    // Useful for checking camera placement and visible tags.
    // The view can be displayed in a browser at http://localhost:1182/?action=stream
    cameraSim.enableProcessedStream(true);

    // If true, adds a field wireframe to the simulated images.	
    // Optional; PhotonVision describes it as extremely resource intensive.
    cameraSim.enableDrawWireframe(false);
    
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}

package frc.robot.subsystems.vision.apriltag;

import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagIOSim extends AprilTagIOHardware {
    public static VisionSystemSim visionSim;

    private final PhotonCameraSim cameraSim;
    private final RobotState robotState;

    public AprilTagIOSim(AprilTagConfig config, RobotState robotState) {
        super(config);

        this.robotState = robotState;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.fieldLayout);
        }

        cameraSim = new PhotonCameraSim(camera, new SimCameraProperties());
        visionSim.addCamera(cameraSim, config.robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        visionSim.update(robotState.getPose());

        super.updateInputs(inputs);
    }
}

package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;

import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.GeometryUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagIOSim implements AprilTagIO {
    public static VisionSystemSim visionSim;

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final AprilTagConfig config;
    private final RobotState robotState;

    public AprilTagIOSim(AprilTagConfig config, RobotState robotState) {
        this.config = config;
        this.robotState = robotState;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.fieldLayout);
        }

        camera = new PhotonCamera(config.cameraName);
        cameraSim = new PhotonCameraSim(camera, new SimCameraProperties());
        visionSim.addCamera(cameraSim, config.robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        visionSim.update(robotState.getPose());

        inputs.isConnected = camera.isConnected();
        inputs.pipelineResults = new AprilTagPipelineResults(camera.getAllUnreadResults().stream()
                .map(this::toPipelineResults)
                .toArray(AprilTagPipelineResult[]::new));
    }

    private AprilTagPipelineResult toPipelineResults(PhotonPipelineResult result) {
        var targets = new AprilTagTarget[result.getTargets().size()];
        Arrays.setAll(targets, i -> {
            var target = result.getTargets().get(i);
            return new AprilTagTarget(
                    target.getFiducialId(),
                    target.getBestCameraToTarget(),
                    target.getAlternateCameraToTarget(),
                    target.getPoseAmbiguity());
        });

        var multiTargetResult = result.multitagResult.map(multiTagResult -> {
            var tagIds = new int[multiTagResult.fiducialIDsUsed.size()];
            Arrays.setAll(tagIds, i -> multiTagResult.fiducialIDsUsed.get(i));
            return new AprilTagMultiTargetResult(
                    tagIds,
                    GeometryUtil.toPose3d(multiTagResult.estimatedPose.best),
                    multiTagResult.estimatedPose.bestReprojErr);
        });

        return new AprilTagPipelineResult(0.0, result.getTimestampSeconds(), targets, multiTargetResult);
    }

    @Override
    public AprilTagConfig getConfig() {
        return config;
    }
}

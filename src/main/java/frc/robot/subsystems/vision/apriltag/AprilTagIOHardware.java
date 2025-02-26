package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;

import frc.robot.util.GeometryUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagIOHardware implements AprilTagIO {
    protected final PhotonCamera camera;
    private final AprilTagConfig config;

    public AprilTagIOHardware(AprilTagConfig config) {
        this.config = config;

        camera = new PhotonCamera(config.cameraName);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {

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

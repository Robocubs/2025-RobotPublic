package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionMeasurement;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class AprilTagCamera {
    private final AprilTagIO io;
    private final AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();
    private final AprilTagConfig config;
    private final RobotState robotState;
    private final Alert disconnectedAlert;
    private final String loggingPrefix;

    public AprilTagCamera(AprilTagIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
        config = io.getConfig();
        disconnectedAlert = new Alert("Vision camera " + config.cameraName + " is disconnected.", AlertType.kWarning);
        loggingPrefix = "Vision/Camera" + config.cameraName;
    }

    public void update(List<VisionMeasurement> measurements) {
        io.updateInputs(inputs);
        Logger.processInputs(loggingPrefix, inputs);
        disconnectedAlert.set(!inputs.isConnected);

        List<Pose3d> targetPoses = new ArrayList<>(inputs.pipelineResults.results.length * 22);
        List<Pose3d> usedTargetPoses = new ArrayList<>(inputs.pipelineResults.results.length * 22);
        List<Pose3d> acceptedRobotPoses = new ArrayList<>(inputs.pipelineResults.results.length);
        List<Pose3d> rejectedRobotPoses = new ArrayList<>(inputs.pipelineResults.results.length);

        for (var pipelineResult : inputs.pipelineResults.results) {
            var robotPose =
                    new Pose3d(robotState.getPose(pipelineResult.timestamp).orElse(robotState.getPose()));

            Stream.of(pipelineResult.targets)
                    .map(target -> robotPose.plus(config.robotToCamera).plus(target.bestCameraToTarget))
                    .forEach(targetPoses::add);

            if (pipelineResult.multiTargetResult.isPresent()) {
                var multiTargetResult = pipelineResult.multiTargetResult.get();
                var observedPose = multiTargetResult
                        .cameraPose
                        .relativeTo(FieldConstants.fieldLayout.getOrigin())
                        .plus(config.cameraToRobot);

                if (isValidPose(observedPose)) {
                    acceptedRobotPoses.add(observedPose);
                    measurements.add(new VisionMeasurement(
                            pipelineResult.timestamp,
                            observedPose,
                            getStdDevs(multiTargetResult.targetIds, multiTargetResult.cameraPose)));
                    IntStream.of(multiTargetResult.targetIds)
                            .mapToObj(FieldConstants.fieldLayout::getTagPose)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .forEach(usedTargetPoses::add);
                    continue;
                }

                rejectedRobotPoses.add(observedPose);
            }

            if (pipelineResult.targets.length == 0) {
                continue;
            }

            var lowestAmbiguityTarget = pipelineResult.targets[0];
            for (var i = 1; i < pipelineResult.targets.length; i++) {
                if (pipelineResult.targets[i].ambiguity < lowestAmbiguityTarget.ambiguity) {
                    lowestAmbiguityTarget = pipelineResult.targets[i];
                }
            }

            FieldConstants.fieldLayout.getTagPose(lowestAmbiguityTarget.id).ifPresent(usedTargetPoses::add);

            var targetPose = FieldConstants.fieldLayout.getTagPose(lowestAmbiguityTarget.id);
            if (targetPose.isEmpty()) {
                continue;
            }

            var observedPose = targetPose
                    .get()
                    .transformBy(lowestAmbiguityTarget.bestCameraToTarget.inverse())
                    .transformBy(config.cameraToRobot);

            if (!isValidPose(observedPose)) {
                continue;
            }

            acceptedRobotPoses.add(observedPose);
            measurements.add(new VisionMeasurement(
                    pipelineResult.timestamp,
                    observedPose,
                    getStdDevs(
                            1,
                            lowestAmbiguityTarget
                                    .bestCameraToTarget
                                    .getTranslation()
                                    .getNorm())));
            usedTargetPoses.add(targetPose.get());
        }

        Logger.recordOutput(loggingPrefix + "/TargetPoses", targetPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/UsedTargetPoses", usedTargetPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/AcceptedPoses", acceptedRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/RejectedPoses", rejectedRobotPoses.toArray(Pose3d[]::new));
    }

    private boolean isValidPose(Pose3d pose) {
        return pose.getX() > fieldBorderMargin
                && pose.getY() > fieldBorderMargin
                && pose.getX() < FieldConstants.fieldLayout.getFieldLength() - fieldBorderMargin
                && pose.getY() < FieldConstants.fieldLayout.getFieldWidth() - fieldBorderMargin
                && pose.getZ() > -zMargin
                && pose.getZ() < zMargin;
    }

    private Vector<N3> getStdDevs(int[] targetIds, Pose3d cameraPose) {
        var distance = IntStream.of(targetIds)
                .boxed()
                .map(FieldConstants.fieldLayout::getTagPose)
                .filter(Optional::isPresent)
                .mapToDouble(targetPose -> targetPose.get().getTranslation().getDistance(cameraPose.getTranslation()))
                .average()
                .orElse(10);
        return getStdDevs(targetIds.length, distance);
    }

    private Vector<N3> getStdDevs(double numTargets, double distance) {
        var linear = linearStdDevBaseline * distance * distance * config.weight / numTargets;
        var rotation = numTargets > 1
                ? angularStdDevBaseline * distance * distance * config.weight / numTargets
                : Double.POSITIVE_INFINITY;
        return VecBuilder.fill(linear, linear, rotation);
    }
}

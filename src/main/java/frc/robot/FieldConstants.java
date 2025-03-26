package frc.robot;

import java.util.Arrays;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import lombok.Builder;

import static edu.wpi.first.units.Units.*;

public final class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Distance fieldLength = Meters.of(fieldLayout.getFieldLength());
    public static final Distance fieldWidth = Meters.of(fieldLayout.getFieldWidth());

    public static final Distance distanceBetweenBranches = Inches.of(12.94);
    public static final Distance halfDistanceBetweenBranches = distanceBetweenBranches.div(2);

    public static final Distance reefFaceLength = Inches.of(36.792600);

    public static final Translation2d fieldCenter = new Translation2d(fieldLength.div(2), fieldWidth.div(2));

    private static final AllianceFieldConstants blueConstants = AllianceFieldConstants.builder()
            .reefCenterPoint(getPointBetweenTags(18, 21))
            .processorPose(getTagPose(16))
            .coralStationLeftPose(getTagPose(13))
            .coralStationRightPose(getTagPose(12))
            .reefBranchPoses(getReefBranchPoses(17, 18, 19, 20, 21, 22))
            .reefAlgaeL2Poses(getTagPoses(17, 19, 21))
            .reefAlgaeL3Poses(getTagPoses(18, 20, 22))
            .build();

    private static final AllianceFieldConstants redConstants = AllianceFieldConstants.builder()
            .reefCenterPoint(getPointBetweenTags(7, 10))
            .processorPose(getTagPose(3))
            .coralStationLeftPose(getTagPose(1))
            .coralStationRightPose(getTagPose(2))
            .reefBranchPoses(getReefBranchPoses(6, 7, 8, 9, 10, 11))
            .reefAlgaeL2Poses(getTagPoses(6, 8, 10))
            .reefAlgaeL3Poses(getTagPoses(7, 9, 11))
            .build();

    public static Translation2d reefCenterPoint() {
        return RobotState.isBlue() ? blueConstants.reefCenterPoint : redConstants.reefCenterPoint;
    }

    public static Pose2d processorPose() {
        return RobotState.isBlue() ? blueConstants.processorPose : redConstants.processorPose;
    }

    public static Pose2d leftCoralStationPose() {
        return RobotState.isBlue() ? blueConstants.leftCoralStationPose : redConstants.leftCoralStationPose;
    }

    public static Pose2d rightCoralStationPose() {
        return RobotState.isBlue() ? blueConstants.rightCoralStationPose : redConstants.rightCoralStationPose;
    }

    public static Pose2d[] reefBranchPoses() {
        return RobotState.isBlue() ? blueConstants.reefBranchPoses : redConstants.reefBranchPoses;
    }

    public static Pose2d[] reefAlgaePoses() {
        return RobotState.isBlue() ? blueConstants.reefAlgaePoses : redConstants.reefAlgaePoses;
    }

    public static Pose2d[] reefAlgaeL2Poses() {
        return RobotState.isBlue() ? blueConstants.reefAlgaeL2Poses : redConstants.reefAlgaeL2Poses;
    }

    public static Pose2d[] reefAlgaeL3Poses() {
        return RobotState.isBlue() ? blueConstants.reefAlgaeL3Poses : redConstants.reefAlgaeL3Poses;
    }

    public static Pose2d robotProcessorPose() {
        return RobotState.isBlue() ? blueConstants.robotProcessorPose : redConstants.robotProcessorPose;
    }

    public static Pose2d robotCoralStationLeftPose() {
        return RobotState.isBlue() ? blueConstants.robotCoralStationLeftPose : redConstants.robotCoralStationLeftPose;
    }

    public static Pose2d robotCoralStationRightPose() {
        return RobotState.isBlue() ? blueConstants.robotCoralStationRightPose : redConstants.robotCoralStationRightPose;
    }

    public static Pose2d[] robotCoralL1Poses() {
        return RobotState.isBlue() ? blueConstants.robotCoralL12Poses : redConstants.robotCoralL12Poses;
    }

    public static Pose2d[] robotCoralL2Poses() {
        return RobotState.isBlue() ? blueConstants.robotCoralL12Poses : redConstants.robotCoralL12Poses;
    }

    public static Pose2d[] robotCoralL3Poses() {
        return RobotState.isBlue() ? blueConstants.robotCoralL3Poses : redConstants.robotCoralL3Poses;
    }

    public static Pose2d[] robotCoralL4Poses() {
        return RobotState.isBlue() ? blueConstants.robotCoralL4Poses : redConstants.robotCoralL4Poses;
    }

    public static Pose2d[] robotReefAlgaePoses() {
        return RobotState.isBlue() ? blueConstants.robotReefAlgaePoses : redConstants.robotReefAlgaePoses;
    }

    public static Pose2d[] robotReefAlgaeL2Poses() {
        return RobotState.isBlue() ? blueConstants.robotReefAlgaeL2Poses : redConstants.robotReefAlgaeL2Poses;
    }

    public static Pose2d[] robotReefAlgaeL3Poses() {
        return RobotState.isBlue() ? blueConstants.robotReefAlgaeL3Poses : redConstants.robotReefAlgaeL3Poses;
    }

    private static Pose2d getTagPose(int id) {
        return fieldLayout.getTagPose(id).map(pose -> pose.toPose2d()).orElse(Pose2d.kZero);
    }

    private static Pose2d[] getTagPoses(int... ids) {
        return IntStream.of(ids).mapToObj(FieldConstants::getTagPose).toArray(Pose2d[]::new);
    }

    private static Translation2d getPointBetweenTags(int id1, int id2) {
        return getTagPose(id1).getTranslation().interpolate(getTagPose(id2).getTranslation(), 0.5);
    }

    private static Pose2d[] getReefBranchPoses(int... tagIds) {
        return IntStream.of(tagIds)
                .mapToObj(FieldConstants::getTagPose)
                .flatMap(pose -> Stream.of(
                        pose.transformBy(new Transform2d(0, halfDistanceBetweenBranches.in(Meters), Rotation2d.kZero)),
                        pose.transformBy(
                                new Transform2d(0, -halfDistanceBetweenBranches.in(Meters), Rotation2d.kZero))))
                .toArray(Pose2d[]::new);
    }

    private static class AllianceFieldConstants {
        private final Translation2d reefCenterPoint;
        private final Pose2d processorPose;
        private final Pose2d leftCoralStationPose;
        private final Pose2d rightCoralStationPose;
        private final Pose2d[] reefBranchPoses;
        private final Pose2d[] reefAlgaePoses;
        private final Pose2d[] reefAlgaeL2Poses;
        private final Pose2d[] reefAlgaeL3Poses;

        private final Pose2d robotProcessorPose;
        private final Pose2d robotCoralStationLeftPose;
        private final Pose2d robotCoralStationRightPose;
        private final Pose2d[] robotCoralL12Poses;
        private final Pose2d[] robotCoralL3Poses;
        private final Pose2d[] robotCoralL4Poses;
        private final Pose2d[] robotReefAlgaePoses;
        private final Pose2d[] robotReefAlgaeL2Poses;
        private final Pose2d[] robotReefAlgaeL3Poses;

        @Builder
        private AllianceFieldConstants(
                Translation2d reefCenterPoint,
                Pose2d processorPose,
                Pose2d coralStationLeftPose,
                Pose2d coralStationRightPose,
                Pose2d[] reefBranchPoses,
                Pose2d[] reefAlgaeL2Poses,
                Pose2d[] reefAlgaeL3Poses) {
            this.reefCenterPoint = reefCenterPoint;
            this.processorPose = processorPose;
            this.leftCoralStationPose = coralStationLeftPose;
            this.rightCoralStationPose = coralStationRightPose;
            this.reefBranchPoses = reefBranchPoses;
            this.reefAlgaePoses = Stream.concat(Arrays.stream(reefAlgaeL2Poses), Arrays.stream(reefAlgaeL3Poses))
                    .toArray(Pose2d[]::new);
            this.reefAlgaeL2Poses = reefAlgaeL2Poses;
            this.reefAlgaeL3Poses = reefAlgaeL3Poses;

            var frontFacingRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters), 0.0, Rotation2d.k180deg);
            var backFacingRobotTransform = new Transform2d(Constants.halfRobotLength.in(Meters), 0.0, Rotation2d.kZero);
            var l2CoralRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters) + 0.12, 0.0, Rotation2d.k180deg);
            var l3CoralRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters) + 0.12, 0.0, Rotation2d.k180deg);
            var l4CoralRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters) + 0.31, 0.0, Rotation2d.k180deg);
            var l3AlgaeRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters) + 0.1, 0.0, Rotation2d.k180deg);
            var processorRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters) + 0.05, 0.0, Rotation2d.k180deg);

            robotProcessorPose = processorPose.transformBy(processorRobotTransform);
            robotCoralStationLeftPose = coralStationLeftPose.transformBy(backFacingRobotTransform);
            robotCoralStationRightPose = coralStationRightPose.transformBy(backFacingRobotTransform);
            robotCoralL12Poses = Stream.of(reefBranchPoses)
                    .map(pose -> pose.transformBy(l2CoralRobotTransform))
                    .toArray(Pose2d[]::new);
            robotCoralL3Poses = Stream.of(reefBranchPoses)
                    .map(pose -> pose.transformBy(l3CoralRobotTransform))
                    .toArray(Pose2d[]::new);
            robotCoralL4Poses = Stream.of(reefBranchPoses)
                    .map(pose -> pose.transformBy(l4CoralRobotTransform))
                    .toArray(Pose2d[]::new);
            robotReefAlgaePoses = Stream.of(reefAlgaePoses)
                    .map(pose -> pose.transformBy(frontFacingRobotTransform))
                    .toArray(Pose2d[]::new);
            robotReefAlgaeL2Poses = Stream.of(reefAlgaeL2Poses)
                    .map(pose -> pose.transformBy(frontFacingRobotTransform))
                    .toArray(Pose2d[]::new);
            robotReefAlgaeL3Poses = Stream.of(reefAlgaeL3Poses)
                    .map(pose -> pose.transformBy(l3AlgaeRobotTransform))
                    .toArray(Pose2d[]::new);
        }
    }
}

package frc.robot;

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
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Distance fieldLength = Meters.of(fieldLayout.getFieldLength());
    public static final Distance fieldWidth = Meters.of(fieldLayout.getFieldWidth());

    public static final Distance distanceBetweenBranches = Inches.of(12.94);
    public static final Distance halfDistanceBetweenBranches = distanceBetweenBranches.div(2);

    public static final Translation2d fieldCenter = new Translation2d(fieldLength.div(2), fieldWidth.div(2));

    private static final AllianceFieldConstants blueConstants = AllianceFieldConstants.builder()
            .reefCenterPoint(getPointBetweenTags(18, 21))
            .processorPose(getTagPose(16))
            .coralStationLeftPose(getTagPose(13))
            .coralStationRightPose(getTagPose(12))
            .reefBranchPoses(getReefBranchPoses(17, 18, 19, 20, 21, 22))
            .reefAlgaePoses(getTagPoses(17, 18, 19, 20, 21, 22))
            .build();

    private static final AllianceFieldConstants redConstants = AllianceFieldConstants.builder()
            .reefCenterPoint(getPointBetweenTags(7, 10))
            .processorPose(getTagPose(3))
            .coralStationLeftPose(getTagPose(1))
            .coralStationRightPose(getTagPose(2))
            .reefBranchPoses(getReefBranchPoses(6, 7, 8, 9, 10, 11))
            .reefAlgaePoses(getTagPoses(6, 7, 8, 9, 10, 11))
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

    public static Pose2d robotProcessorPose() {
        return RobotState.isBlue() ? blueConstants.robotProcessorPose : redConstants.robotProcessorPose;
    }

    public static Pose2d robotCoralStationLeftPose() {
        return RobotState.isBlue() ? blueConstants.robotCoralStationLeftPose : redConstants.robotCoralStationLeftPose;
    }

    public static Pose2d robotCoralStationRightPose() {
        return RobotState.isBlue() ? blueConstants.robotCoralStationRightPose : redConstants.robotCoralStationRightPose;
    }

    public static Pose2d[] robotReefBranchPoses() {
        return RobotState.isBlue() ? blueConstants.robotReefBranchPoses : redConstants.robotReefBranchPoses;
    }

    public static Pose2d[] robotReefAlgaePoses() {
        return RobotState.isBlue() ? blueConstants.robotReefAlgaePoses : redConstants.robotReefAlgaePoses;
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

        private final Pose2d robotProcessorPose;
        private final Pose2d robotCoralStationLeftPose;
        private final Pose2d robotCoralStationRightPose;
        private final Pose2d[] robotReefBranchPoses;
        private final Pose2d[] robotReefAlgaePoses;

        @Builder
        private AllianceFieldConstants(
                Translation2d reefCenterPoint,
                Pose2d processorPose,
                Pose2d coralStationLeftPose,
                Pose2d coralStationRightPose,
                Pose2d[] reefBranchPoses,
                Pose2d[] reefAlgaePoses) {
            this.reefCenterPoint = reefCenterPoint;
            this.processorPose = processorPose;
            this.leftCoralStationPose = coralStationLeftPose;
            this.rightCoralStationPose = coralStationRightPose;
            this.reefBranchPoses = reefBranchPoses;
            this.reefAlgaePoses = reefAlgaePoses;

            var frontFacingRobotTransform =
                    new Transform2d(Constants.halfRobotLength.in(Meters), 0.0, Rotation2d.k180deg);
            var backFacingRobotTransform = new Transform2d(Constants.halfRobotLength.in(Meters), 0.0, Rotation2d.kZero);

            robotProcessorPose = processorPose.transformBy(frontFacingRobotTransform);
            robotCoralStationLeftPose = coralStationLeftPose.transformBy(backFacingRobotTransform);
            robotCoralStationRightPose = coralStationRightPose.transformBy(backFacingRobotTransform);
            robotReefBranchPoses = Stream.of(reefBranchPoses)
                    .map(pose -> pose.transformBy(frontFacingRobotTransform))
                    .toArray(Pose2d[]::new);
            robotReefAlgaePoses = Stream.of(reefAlgaePoses)
                    .map(pose -> pose.transformBy(frontFacingRobotTransform))
                    .toArray(Pose2d[]::new);
        }
    }
}

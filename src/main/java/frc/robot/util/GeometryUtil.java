package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

public class GeometryUtil {
    public static Rotation2d flipX(Rotation2d rotation) {
        return Rotation2d.k180deg.minus(rotation);
    }

    public static Translation2d flipX(Translation2d translation) {
        return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
    }

    public static Pose2d flipX(Pose2d pose) {
        return new Pose2d(flipX(pose.getTranslation()), flipX(pose.getRotation()));
    }

    public static Pose2d[] flipX(Pose2d[] poses) {
        var flippedPoses = new Pose2d[poses.length];
        for (var i = 0; i < poses.length; i++) {
            flippedPoses[i] = flipX(poses[i]);
        }

        return flippedPoses;
    }

    public static Rotation2d autoFlipX(Rotation2d rotation) {
        return RobotState.isBlue() ? rotation : flipX(rotation);
    }

    public static Translation2d autoFlipX(Translation2d translation) {
        return RobotState.isBlue() ? translation : flipX(translation);
    }

    public static Pose2d autoFlipX(Pose2d pose) {
        return RobotState.isBlue() ? pose : flipX(pose);
    }

    public static Pose2d[] autoFlipX(Pose2d[] poses) {
        return RobotState.isBlue() ? poses : flipX(poses);
    }

    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }
}

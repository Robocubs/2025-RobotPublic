package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

import static edu.wpi.first.units.Units.Meters;

public final class GeometryUtil {
    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.minus(Rotation2d.kPi);
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
                FieldConstants.fieldLength.in(Meters) - translation.getX(),
                FieldConstants.fieldWidth.in(Meters) - translation.getY());
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    public static Pose2d[] flip(Pose2d[] poses) {
        var flippedPoses = new Pose2d[poses.length];
        for (var i = 0; i < poses.length; i++) {
            flippedPoses[i] = flip(poses[i]);
        }

        return flippedPoses;
    }

    public static Rotation2d autoFlip(Rotation2d rotation) {
        return RobotState.isBlue() ? rotation : flip(rotation);
    }

    public static Translation2d autoFlip(Translation2d translation) {
        return RobotState.isBlue() ? translation : flip(translation);
    }

    public static Pose2d autoFlip(Pose2d pose) {
        return RobotState.isBlue() ? pose : flip(pose);
    }

    public static Pose2d[] autoFlip(Pose2d[] poses) {
        return RobotState.isBlue() ? poses : flip(poses);
    }

    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.apriltag.AprilTagConfig;

public final class VisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double fieldBorderMargin = 0.0;
    public static final double zMargin = 0.75;
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    public static final SingleTargetMode singleTargetMode = SingleTargetMode.LOWEST_AMBIGUITY;

    public static enum SingleTargetMode {
        LOWEST_AMBIGUITY,
        CLOSEST_TO_HEADING,
    }

    public static AprilTagConfig frontAprilTagConfig =
            new AprilTagConfig("Front", new Transform3d(0.1, 0.3, 0.4, Rotation3d.kZero));
    public static AprilTagConfig backAprilTagConfig =
            new AprilTagConfig("Back", new Transform3d(-0.1, -0.3, 0.4, new Rotation3d(Rotation2d.k180deg)));
}

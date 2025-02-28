package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.apriltag.AprilTagConfig;

import static edu.wpi.first.units.Units.*;

public final class VisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double fieldBorderMargin = 0.0;
    public static final double zMargin = 0.75;
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.01; // Radians

    public static final SingleTargetMode singleTargetMode = SingleTargetMode.LOWEST_AMBIGUITY;

    public static enum SingleTargetMode {
        LOWEST_AMBIGUITY,
        CLOSEST_TO_HEADING,
    }

    public static AprilTagConfig frontLeftAprilTagConfig = new AprilTagConfig(
            "FL",
            new Transform3d(
                    Inches.of(13.5),
                    Inches.of(5.5),
                    Inches.of(6.5),
                    new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.zero())));
    public static AprilTagConfig frontRightAprilTagConfig = new AprilTagConfig(
            "FR",
            new Transform3d(
                    Inches.of(13.5),
                    Inches.of(-5.5),
                    Inches.of(6.5),
                    new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.zero())));
}

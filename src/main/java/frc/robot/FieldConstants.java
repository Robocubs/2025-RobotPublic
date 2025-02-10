package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

public final class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Distance fieldLength = Meters.of(fieldLayout.getFieldLength());
    public static final Distance fieldWidth = Meters.of(fieldLayout.getFieldWidth());
}

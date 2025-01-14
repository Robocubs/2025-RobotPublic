package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final double reduction = 5.0;
    public static final Rotation2d angleTolerance = Rotation2d.fromDegrees(1.0);
    public static final Rotation2d minimumAngle = Rotation2d.fromDegrees(90);
    public static final Rotation2d maximumAngle = Rotation2d.fromDegrees(270);
}

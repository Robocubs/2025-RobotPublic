package frc.robot.subsystems.superstructure.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.CustomDCMotor;

import static edu.wpi.first.units.Units.*;

public final class RollersConstants {
    public static final Distance coralRollerRadius = Inches.of(4);
    public static final Distance algaeRollerRadius = Inches.of(1);
    public static final double reduction = 9.0 * 24.0 / 20.0 * 24.0 / 19.0 * 24.0 / 22.0;
    public static final Angle coralAngleRelativeToArm = Degrees.of(-20.855);
    public static final Angle algaeAngleRelativeToArm = Degrees.of(10.332);

    public static final AngularVelocity maxAngularVelocity =
            RadiansPerSecond.of(CustomDCMotor.getKrakenX44(1).withReduction(reduction).freeSpeedRadPerSec * 0.8);
    public static final double kT = CustomDCMotor.getKrakenX44(1).withReduction(reduction).KtNMPerAmp;

    public static final Distance coralDetectionDistance = Meters.of(0.25);
    public static final Distance algaeDetectionDistance = Meters.of(0.02);
    public static final Distance funnelDetectionDistance = Meters.of(0.25);
    public static final Distance elevatorDetectionDistance = Meters.of(0.55);
    public static final Distance elevatorMaxHeightForDetection = Meters.of(0.2);
    public static final Distance detectionDistanceTolerance = Meters.of(0.01);
}

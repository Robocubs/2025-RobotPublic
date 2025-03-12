package frc.robot.subsystems.superstructure.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.CustomDCMotor;

import static edu.wpi.first.units.Units.*;

public final class RollersConstants {
    public static final Distance coralRollerRadius = Inches.of(1.2);
    public static final Distance hybridRollerRadius = Inches.of(0.6);
    public static final double coralRollerReduction = 9.0 * 22.0 / 24.0;
    public static final double hybridRollerReduction = 5.0 * 24.0 / 18.0;
    public static final Angle coralAngleRelativeToArm = Degrees.of(-20.855);
    public static final Angle algaeAngleRelativeToArm = Degrees.of(10.332);

    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(Math.min(
                    CustomDCMotor.getKrakenX44(1).withReduction(coralRollerReduction).freeSpeedRadPerSec
                            * coralRollerRadius.in(Meters),
                    CustomDCMotor.getKrakenX44(1).withReduction(hybridRollerReduction).freeSpeedRadPerSec
                            * hybridRollerRadius.in(Meters))
            * 0.8);
    public static final AngularVelocity coralRollerMaxVelocity =
            RadiansPerSecond.of(maxLinearVelocity.in(MetersPerSecond) / coralRollerRadius.in(Meters));
    public static final AngularVelocity hybridRollerMaxVelocity =
            RadiansPerSecond.of(maxLinearVelocity.in(MetersPerSecond) / hybridRollerRadius.in(Meters));

    public static final double kTCoralRoller =
            CustomDCMotor.getKrakenX44(1).withReduction(coralRollerReduction).KtNMPerAmp;
    public static final double kTHybridRoller =
            CustomDCMotor.getKrakenX44(1).withReduction(hybridRollerReduction).KtNMPerAmp;

    public static final Distance coralDetectionDistance = Meters.of(0.3);
    public static final Distance algaeDetectionDistance = Meters.of(0.3);
    public static final Distance elevatorDetectionDistance = Meters.of(0.55);
    public static final Distance elevatorMaxHeightForDetection = Meters.of(0.2);
    public static final Distance detectionDistanceTolerance = Meters.of(0.04);

    public static final Distance coralFeedDistance = Meters.of(0.3);
    public static final Time coralFeedTime = Seconds.of(0.5);
    public static final Angle coralFeedCoralRollerPosition =
            Radians.of(coralFeedDistance.in(Meters) / coralRollerRadius.in(Meters));
    public static final Angle coralFeedHybridRollerPosition =
            Radians.of(-coralFeedDistance.in(Meters) / hybridRollerRadius.in(Meters));
    public static final AngularVelocity coralFeedCoralRollerVelocity = coralFeedCoralRollerPosition.div(coralFeedTime);
    public static final AngularVelocity coralFeedHybridRollerVelocity =
            coralFeedHybridRollerPosition.div(coralFeedTime);
    public static final AngularVelocity coralFeedFastCoralRollerVelocity = coralFeedCoralRollerVelocity.times(1.3);
    public static final AngularVelocity coralFeedFastHybridRollerVelocity = coralFeedHybridRollerVelocity.times(1.3);

    public static final Distance coralIntakeDistance = Inches.of(-3);
    public static final Time coralIntakeTime = Seconds.of(0.1);
    public static final Angle coralIntakeCoralRollerPosition =
            Radians.of(coralIntakeDistance.in(Meters) / coralRollerRadius.in(Meters));
    public static final Angle coralIntakeHybridRollerPosition =
            Radians.of(-coralIntakeDistance.in(Meters) / hybridRollerRadius.in(Meters));
    public static final AngularVelocity coralIntakeCoralRollerVelocity =
            coralIntakeCoralRollerPosition.div(coralIntakeTime);
    public static final AngularVelocity coralIntakeHybridRollerVelocity =
            coralIntakeHybridRollerPosition.div(coralIntakeTime);

    public static final Distance algaeIntakeDistance = Inches.of(-16);
    public static final Time algaeIntakeTime = Seconds.of(0.2);
    public static final Angle algaeIntakeCoralRollerPosition =
            Radians.of(algaeIntakeDistance.in(Meters) / coralRollerRadius.in(Meters));
    public static final Angle algaeIntakeHybridRollerPosition =
            Radians.of(algaeIntakeDistance.in(Meters) / hybridRollerRadius.in(Meters));
    public static final AngularVelocity algaeIntakeCoralRollerVelocity =
            algaeIntakeCoralRollerPosition.div(algaeIntakeTime);
    public static final AngularVelocity algaeIntakeHybridRollerVelocity =
            algaeIntakeHybridRollerPosition.div(algaeIntakeTime);

    public static final AngularVelocity coralForwardCoralRollerVelocity = coralIntakeCoralRollerVelocity.times(-3);
    public static final AngularVelocity coralForwardHybridRollerVelocity = algaeIntakeHybridRollerVelocity.times(-3);
    public static final AngularVelocity coralReverseCoralRollerVelocity = coralIntakeCoralRollerVelocity.times(3);
    public static final AngularVelocity coralReverseHybridRollerVelocity = algaeIntakeHybridRollerVelocity.times(3);

    public static final AngularVelocity algaeForwardCoralRollerVelocity = algaeIntakeCoralRollerVelocity.times(-1);
    public static final AngularVelocity algaeForwardHybridRollerVelocity = algaeIntakeHybridRollerVelocity.times(-1);
}

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class SuperstructureConstants {
    public static final Transform3d robotToElevator = new Transform3d(
            Inches.of(4.254),
            Inches.zero(),
            Inches.of(1.437),
            new Rotation3d(Degrees.zero(), ElevatorConstants.elevatorAngle.times(-1), Degrees.zero()));
    public static final Transform3d elevatorHeightToCarriage = new Transform3d(
            Inches.of(8.5),
            Inches.zero(),
            Inches.zero(),
            new Rotation3d(Degrees.zero(), Degrees.of(90.0), Degrees.zero()));
    public static final Translation3d carriageToArmMount =
            new Translation3d(Inches.of(7.562), Inches.zero(), Inches.zero());
    public static final Transform3d armMountToArmEnd = new Transform3d(
            ArmConstants.length,
            Inches.zero(),
            Inches.zero(),
            new Rotation3d(Degrees.zero(), Degrees.of(20.855), Degrees.zero()));

    public static final Distance maximumHeight = Pose3d.kZero
            .transformBy(robotToElevator)
            .transformBy(
                    new Transform3d(ElevatorConstants.maximumHeight, Inches.zero(), Inches.zero(), Rotation3d.kZero))
            .transformBy(elevatorHeightToCarriage)
            .transformBy(new Transform3d(
                    carriageToArmMount,
                    new Rotation3d(
                            Degrees.zero(), Degrees.of(-180).plus(ElevatorConstants.elevatorAngle), Degrees.zero())))
            .transformBy(armMountToArmEnd)
            .getMeasureZ();
}

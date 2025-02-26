package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.*;

public record SuperstructurePose(Distance elevatorHeight, Angle armAngle) {
    public SuperstructurePose() {
        this(Inches.zero(), Degrees.of(90));
    }

    public Transform3d getRobotToEndEffector() {
        return new Transform3d(
                Pose3d.kZero,
                Pose3d.kZero
                        .transformBy(SuperstructureConstants.robotToElevator)
                        .transformBy(new Transform3d(elevatorHeight.in(Meters), 0.0, 0.0, Rotation3d.kZero))
                        .transformBy(SuperstructureConstants.elevatorHeightToCarriage)
                        .transformBy(new Transform3d(
                                SuperstructureConstants.carriageToArmMount,
                                new Rotation3d(0.0, -armAngle.in(Radians), 0.0)))
                        .transformBy(SuperstructureConstants.armMountToArmEnd));
    }

    @RequiredArgsConstructor
    @Getter
    public static enum Preset {
        STOW(Inches.zero(), ArmConstants.safeTravelAngle),
        L1_LONG(Inches.zero(), Degrees.of(90)),
        L1_WIDE(Inches.of(2), Degrees.of(70)),
        L2(Inches.of(12), Degrees.of(85)),
        L3(Inches.of(28), Degrees.of(85)),
        L4(Inches.of(61), Degrees.of(42.819)),
        BARGE(ElevatorConstants.maximumHeight, Degrees.of(90)),
        L2_ALGAE(Inches.of(17.5), Degrees.of(75)),
        L3_ALGAE(Inches.of(34.0), Degrees.of(80)),
        ALGAE_INTAKE(Inches.zero(), Degrees.of(20.0)),
        CORAL_INTAKE_1(Inches.of(11.288), Degrees.of(-55.752)),
        CORAL_INTAKE_2(Inches.of(8.198), Degrees.of(-60.801)),
        FEED(Inches.zero(), Degrees.of(92.24)),
        PROCESSOR(Inches.zero(), Degrees.of(80));

        private final SuperstructurePose pose;
        private final SuperstructurePose retractPose;

        private Preset(Distance elevatorHeight, Angle armAngle) {
            this(
                    new SuperstructurePose(elevatorHeight, armAngle),
                    new SuperstructurePose(elevatorHeight, ArmConstants.safeTravelAngle));
        }
    }
}

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.*;

public record SuperstructurePose(Distance elevatorHeight, Angle armAngle) {
    public SuperstructurePose() {
        this(Inches.zero(), ArmConstants.safeTravelAngle);
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
        STOW(Inches.zero(), ArmConstants.safeTravelAngle, ArmConstants.algaeSafeTravelAngle),
        L1(Inches.of(5), Degrees.of(80)),
        L2(Inches.of(16.5), Degrees.of(75)),
        L3(Inches.of(30.5), Degrees.of(75)),
        L4(Inches.of(60), Degrees.of(42.819)),
        BARGE(Inches.of(79), Degrees.of(70)),
        L2_ALGAE(Inches.of(21), Degrees.of(60)),
        L3_ALGAE(Inches.of(36), Degrees.of(60)),
        ALGAE_INTAKE(Inches.zero(), Degrees.of(15.0)),
        CORAL_INTAKE(Inches.of(7.5), Degrees.of(-65)),
        FEED(Inches.of(0), Degrees.of(88.8)),
        PROCESSOR(Inches.of(2), ArmConstants.algaeSafeTravelAngle);

        private final SuperstructurePose pose;
        private final SuperstructurePose retractPose;
        private final SuperstructurePose algaePose;
        private final SuperstructurePose algaeRetractPose;

        private Preset(Distance elevatorHeight, Angle armAngle) {
            this(elevatorHeight, armAngle, armAngle);
        }

        private Preset(Distance elevatorHeight, Angle armAngle, Angle algaeArmAngle) {
            this(
                    new SuperstructurePose(elevatorHeight, armAngle),
                    new SuperstructurePose(elevatorHeight, ArmConstants.safeTravelAngle),
                    new SuperstructurePose(elevatorHeight, algaeArmAngle),
                    new SuperstructurePose(elevatorHeight, ArmConstants.algaeSafeTravelAngle));
        }
    }
}

package frc.robot.subsystems.superstructure;

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

    @RequiredArgsConstructor
    @Getter
    public static enum Preset {
        STOW(Inches.zero(), Degrees.of(90)),
        L1_LONG(Inches.zero(), Degrees.of(90)),
        L1_WIDE(Inches.of(1.894), Degrees.of(70)),
        L2(Inches.of(9.523), Degrees.of(87.021)),
        L3(Inches.of(24.939), Degrees.of(92.848)),
        L4(Inches.of(59.689), Degrees.of(42.819)),
        BARGE(ElevatorConstants.maximumHeight, Degrees.of(105)),
        L2_ALGAE(Inches.of(34.0), Degrees.of(80)),
        L3_ALGAE(Inches.of(17.5), Degrees.of(75)),
        ALGAE_INTAKE(Inches.of(0.187), Degrees.of(25.0)),
        CORAL_INTAKE_1(Inches.of(11.288), Degrees.of(-55.752)),
        CORAL_INTAKE_2(Inches.of(8.198), Degrees.of(-60.801)),
        FEED(Inches.of(1.838), Degrees.of(83.364)),
        PROCESSOR(Inches.zero(), Degrees.of(90));

        private final SuperstructurePose pose;
        private final SuperstructurePose retractPose;

        private Preset(Distance elevatorHeight, Angle armAngle) {
            this(
                    new SuperstructurePose(elevatorHeight, armAngle),
                    new SuperstructurePose(elevatorHeight, ArmConstants.safeTravelAngle));
        }
    }
}

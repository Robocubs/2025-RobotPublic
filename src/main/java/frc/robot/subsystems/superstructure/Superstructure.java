package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;

public class Superstructure extends SubsystemBase {
    private static final double armAngleOffsetDegrees = 90 - ElevatorConstants.elevatorAngle.in(Degrees);

    private final Elevator elevator;
    private final Arm arm;
    private final RobotState robotState;

    @AutoLogOutput
    private final LoggedMechanism2d mechanism;

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;

    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public static enum Goal {
        // Elevator height 0 is fully retracted
        // Arm angle 0 is horizontal to ground
        STOP(),
        HOLD(),
        STOW(Inches.zero(), Degrees.of(90)),
        L1_LONG(Inches.zero(), Degrees.of(90)),
        L1_WIDE(Inches.of(1.894), Degrees.of(70)),
        L2(Inches.of(9.523), Degrees.of(87.021)),
        L3(Inches.of(24.939), Degrees.of(92.848)),
        L4(Inches.of(59.689), Degrees.of(42.819)),
        BARGE(Inches.of(81.161), Degrees.of(105)),
        L2_ALGAE(Inches.of(34.0), Degrees.of(80)),
        L3_ALGAE(Inches.of(17.5), Degrees.of(75)),
        ALGAE_INTAKE(Inches.of(0.187), Degrees.of(25.0)),
        CORAL_INTAKE_1(Inches.of(11.288), Degrees.of(-55.752)),
        CORAL_INTAKE_2(Inches.of(8.198), Degrees.of(-60.801)),
        FUNNEL(Inches.of(1.838), Degrees.of(83.364)),
        PROCESSOR(Inches.zero(), Degrees.of(90));

        public final Distance elevatorHeight;
        public final Angle armAngle;

        private Goal() {
            this.elevatorHeight = Inches.zero();
            this.armAngle = Degrees.zero();
        }

        private Goal(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }

    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, RobotState robotState) {
        elevator = new Elevator(elevatorIO);
        arm = new Arm(armIO);
        this.robotState = robotState;

        mechanism = new LoggedMechanism2d(1.0, 3.0);

        // TODO: Load values from config file
        var root = mechanism.getRoot("Superstructure", Units.inchesToMeters(4.254), Units.inchesToMeters(1.437));
        elevatorLigament = root.append(new LoggedMechanismLigament2d(
                "Elevator", elevator.getHeight().in(Meters), ElevatorConstants.elevatorAngle.in(Degrees)));
        armLigament = elevatorLigament
                .append(new LoggedMechanismLigament2d("Carriage", Units.inchesToMeters(8.5), 0))
                .append(new LoggedMechanismLigament2d("ArmMount", Units.inchesToMeters(7.562), -90))
                .append(new LoggedMechanismLigament2d("Arm", Units.inchesToMeters(17.757), 95));
        armLigament.setColor(new Color8Bit(Color.kBlue));
    }

    @Override
    public void periodic() {
        elevator.periodic();
        arm.periodic();

        elevatorLigament.setLength(elevator.getHeight().in(Meters));
        armLigament.setAngle(arm.getAngle().in(Degrees) + armAngleOffsetDegrees);
    }

    @AutoLogOutput
    public boolean atGoal() {
        // TODO: Return whether the elevator and arm are both at the goal
        return false;
    }

    public void setGoal(Goal goal) {
        // TODO: Set the goal for the elevator and arm
    }

    public Command moveToGoal(Supplier<Goal> goal) {
        /*
         * TODO: Implement the following logic:
         * 1. If the superstructure is not at the goal, but the arm is extended, retract the arm while holding elevator position
         * 2. Move the elevator to the goal position
         * 3. Move to the final goal
         *
         * Examples:
         * - Moving from L2_INTAKE to L3_INTAKE
         *   1. Set elevator to hold position and arm to STOW
         *   2. Set elevator to L3
         *   3. Set arm to REEF_INTAKE
         * - Moving from STOW to FLOOR_INTAKE
         *   1. Set elevator to hold position and arm to STOW
         *   2. Set elevator to L3
         *   3. Set arm to REEF_INTAKE
         */
        return print("Moving superstructure to " + goal.get()).withName("SuperstructureSetGoal");
    }

    public Command hold() {
        // TODO: Hold the current position of the elevator and arm
        // Set the goal to hold
        return print("Holding superstructure").withName("SuperstructureHold");
    }

    public Command stop() {
        // TODO: Stop elevator and arm
        // Set the goal to stop
        return print("Stopping superstructure").withName("SuperstructureStop");
    }
}

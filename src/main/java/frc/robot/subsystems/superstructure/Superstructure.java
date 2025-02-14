package frc.robot.subsystems.superstructure;

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
import frc.robot.subsystems.superstructure.controllers.GraphController;
import frc.robot.subsystems.superstructure.controllers.SuperstructureController;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.rollers.Rollers;
import frc.robot.subsystems.superstructure.rollers.RollersIO;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;

public class Superstructure extends SubsystemBase {
    private static final double armAngleOffsetDegrees = 90 - ElevatorConstants.elevatorAngle.in(Degrees);

    private final Elevator elevator;
    private final Arm arm;
    private final Rollers rollers;
    private final RobotState robotState;
    private final SuperstructureController controller = new GraphController(this);

    @AutoLogOutput
    private final LoggedMechanism2d mechanism;

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;

    @AutoLogOutput
    @Getter
    private SuperstructureState state = SuperstructureState.START;

    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, RollersIO rollersIO, RobotState robotState) {
        elevator = new Elevator(elevatorIO);
        arm = new Arm(armIO);
        rollers = new Rollers(rollersIO, arm::getAngle);
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
        rollers.periodic();

        elevatorLigament.setLength(elevator.getHeight().in(Meters));
        armLigament.setAngle(arm.getAngle().in(Degrees) + armAngleOffsetDegrees);
    }

    public Distance getElevatorHeight() {
        // TODO: Return the elevator height
        return Inches.zero();
    }

    public Angle getArmAngle() {
        // TODO: Return the arm angle
        return Degrees.zero();
    }

    public boolean isNear(SuperstructurePose pose) {
        // TODO: Return whether the elevator and arm are both at the pose
        return false;
    }

    public boolean isNear(SuperstructureState state) {
        // TODO: Return whether the elevator and arm are both at the state's pose
        return false;
    }

    @AutoLogOutput
    public boolean atStatePose() {
        // TODO: Return whether the elevator and arm are both at the state's pose
        return false;
    }

    public void setState(SuperstructureState state) {
        /*
         * Always set the roller state to the state data's roller state
         *
         * If the state is start, command the elevator and arm to hold
         * If the state is stop, command the elevator and arm to stop
         * If the state is hold, command the elevator and arm to hold
         * If the state is retract arm, command the elevator to hold and the arm to the state's pose
         * If the state is zero elevator, do not command the elevator but command the arm to the state's pose
         *
         * Otherwise,
         * 1. Set the elevator height and arm angle to the state's pose
         * 2. Set the rollers to the state's roller state
         */
    }

    public Command runState(SuperstructureState state) {
        /*
         * TODO: Implement the following logic
         * 1. If the goal is STOP, return stop command
         * 2. If the goal is HOLD, return hold command
         * 3. If the goal is RETRACT_ARM, return retractArm command
         * 4. Otherwise, set the goal and return the command from the controller
         *
         * Notes: The goal needs to be set by the command, which could be a runOnce
         * The controller can be used to get the command for moving to the goal
         */
        return print("Running state " + state).withName("SuperstructureRunState");
    }

    public Command hold() {
        // TODO: Set the state to HOLD with a runOnce command
        return print("Holding superstructure").withName("SuperstructureHold");
    }

    public Command stop() {
        // TODO: Set the state to STOP with a runOnce command
        return print("Stopping superstructure").withName("SuperstructureStop");
    }

    public Command retractArm() {
        // TODO: Set the state to RETRACT_ARM with a runOnce command
        return print("Stopping superstructure").withName("SuperstructureRetractArm");
    }

    public Command zeroElevator() {
        return runOnce(() -> setState(SuperstructureState.ZERO_ELEVATOR))
                .andThen(elevator.zero())
                .finallyDo(() -> setState(SuperstructureState.STOW))
                .withName("SuperstructureZeroElevator");
    }
}

package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
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
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.idle;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

public class Superstructure extends SubsystemBase {
    private static final double armAngleOffsetDegrees = 90 - ElevatorConstants.elevatorAngle.in(Degrees);
    private static final LinearVelocity elevatorZeroMinVelocity = InchesPerSecond.of(-0.01);

    private final Elevator elevator;
    private final Arm arm;
    private final Rollers rollers;
    private final RobotState robotState;
    private final SuperstructureController controller = new GraphController(this);

    private final @AutoLogOutput LoggedMechanism2d mechanism;

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;

    private @AutoLogOutput @Getter SuperstructureState state = SuperstructureState.START;
    private @AutoLogOutput boolean elevatorZeroed = false;

    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, RollersIO rollersIO, RobotState robotState) {
        elevator = new Elevator(elevatorIO);
        arm = new Arm(armIO);
        rollers = new Rollers(rollersIO);
        this.robotState = robotState;

        mechanism = new LoggedMechanism2d(1.0, 3.0);

        var root = mechanism.getRoot("Superstructure", robotToElevator.getX(), robotToElevator.getZ());
        elevatorLigament = root.append(new LoggedMechanismLigament2d(
                "Elevator", elevator.getHeight().in(Meters), ElevatorConstants.elevatorAngle.in(Degrees)));
        armLigament = elevatorLigament
                .append(new LoggedMechanismLigament2d("Carriage", elevatorHeightToCarriage.getX(), 0))
                .append(new LoggedMechanismLigament2d(
                        "ArmMount",
                        carriageToArmMount.getX(),
                        Units.radiansToDegrees(
                                -elevatorHeightToCarriage.getRotation().getY())))
                .append(new LoggedMechanismLigament2d("Arm", ArmConstants.length.in(Meters), 95));
        armLigament.setColor(new Color8Bit(Color.kBlue));

        var zeroElevator = sequence(
                        waitSeconds(1.0),
                        idle().until(() -> elevator.getVelocity().gt(elevatorZeroMinVelocity)),
                        elevator.setZeroPosition(),
                        runOnce(() -> elevatorZeroed = true))
                .unless(() -> elevatorZeroed);
        var disabled = sequence(stop(), zeroElevator).ignoringDisable(true).withName("SuperstructureDisbaled");
        disabled.schedule();
        RobotModeTriggers.disabled().whileTrue(disabled);
    }

    @Override
    public void periodic() {
        elevator.updateInputs();
        arm.updateInputs();
        rollers.updateInputs();

        elevatorLigament.setLength(elevator.getHeight().in(Meters));
        armLigament.setAngle(arm.getAngle().in(Degrees) + armAngleOffsetDegrees);

        robotState.setGamePieceStates(
                rollers.longCoralDetected(), rollers.wideCoralDetected(), rollers.algaeDetected());

        robotState.updateSuperstructureState(
                state,
                atStatePose()
                        ? state.getData().getPose()
                        : new SuperstructurePose(elevator.getHeight(), arm.getAngle()));
    }

    public Distance getElevatorHeight() {
        return elevator.getHeight();
    }

    public Angle getArmAngle() {
        return arm.getAngle();
    }

    public boolean isNear(SuperstructurePose pose) {
        return arm.isNear(pose.armAngle()) && elevator.isNear(pose.elevatorHeight());
    }

    public boolean isNear(SuperstructureState state) {
        switch (state) {
            case START:
            case STOP:
            case HOLD:
                return true;
            case RETRACT_ARM:
                return arm.isNear(state.getData().getPose().armAngle());
            default:
                return isNear(state.getData().getPose());
        }
    }

    @AutoLogOutput
    public boolean atStatePose() {
        return isNear(state);
    }

    public void setState(SuperstructureState state) {
        runStatePeriodic(state);
    }

    public void runStatePeriodic(SuperstructureState state) {
        this.state = state;

        rollers.runState(state.getData().getRollerState());

        switch (state) {
            case START:
                elevator.hold();
                arm.hold();
                break;
            case STOP:
                elevator.stop();
                arm.stop();
                break;
            case HOLD:
                elevator.hold();
                arm.hold();
                break;
            case RETRACT_ARM:
                elevator.hold();
                arm.setAngle(state.getData().getPose().armAngle());
                break;
            case ZERO_ELEVATOR:
                arm.setAngle(state.getData().getPose().armAngle());
                break;
            default:
                elevator.setHeight(state.getData().getPose().elevatorHeight());
                arm.setAngle(state.getData().getPose().armAngle());
                break;
        }
    }

    public Command transitionToState(SuperstructureState state) {
        return transitionToState(state, false);
    }

    public Command transitionToState(SuperstructureState state, boolean maintainState) {
        switch (state) {
            case START:
                Commands.print("START is not intended to be run")
                        .andThen(hold())
                        .withName("SuperstructureStart");
            case STOP:
                return stop(maintainState);
            case HOLD:
                return hold(maintainState);
            case RETRACT_ARM:
                return retractArm(maintainState);
            case ZERO_ELEVATOR:
                return zeroElevator();
            default:
                return defer(() -> controller.getCommand(this.state, state))
                        .withName("SuperstructureRunState_" + state.name());
        }
    }

    public Command maintainState() {
        return run(() -> runStatePeriodic(state)).withName("SuperstructureMaintainState" + state.name());
    }

    public Command hold() {
        return hold(false);
    }

    public Command hold(boolean maintainState) {
        return (maintainState
                        ? run(() -> runStatePeriodic(SuperstructureState.HOLD))
                        : runOnce(() -> runStatePeriodic(SuperstructureState.HOLD)))
                .withName("SuperstructureHold");
    }

    public Command stop() {
        return hold(false);
    }

    public Command stop(boolean maintainState) {
        return (maintainState
                        ? run(() -> runStatePeriodic(SuperstructureState.STOP))
                        : runOnce(() -> runStatePeriodic(SuperstructureState.STOP)))
                .withName("SuperstructureStop");
    }

    public Command retractArm() {
        return retractArm(false);
    }

    public Command retractArm(boolean maintainState) {
        return run(() -> runStatePeriodic(SuperstructureState.RETRACT_ARM))
                .until(() -> !maintainState
                        && arm.isNear(SuperstructureState.RETRACT_ARM
                                .getData()
                                .getPose()
                                .armAngle()))
                .withName("SuperstructureRetractArm");
    }

    public Command score(SuperstructureState prescoreState, SuperstructureState scoreState, BooleanSupplier release) {
        // TODO: Ensure the coral doesn't get partially scored
        return sequence(
                        // Wait for the robot to be at the correct speed
                        hold().andThen(maintainState())
                                .until(() -> robotState.robotSpeedNominal(prescoreState))
                                .unless(() -> robotState.robotSpeedNominal(prescoreState)),
                        // Move to the scoring position and wait for release button
                        transitionToState(prescoreState),
                        maintainState().until(release),
                        // Score the game piece
                        transitionToState(scoreState),
                        maintainState())
                .withName("SuperstructureScore_" + scoreState.name());
    }

    public Command runState(SuperstructureState state) {
        return sequence(
                        // Wait for the robot to be at the correct speed
                        hold(true)
                                .until(() -> robotState.robotSpeedNominal(state))
                                .unless(() -> robotState.robotSpeedNominal(state)),
                        // Move to the scoring position and wait for release button
                        transitionToState(state),
                        maintainState())
                .withName("SuperstructureScore_" + state.name());
    }

    public Command zeroElevator() {
        return sequence(
                        retractArm(),
                        deadline(
                                elevator.zeroRoutine(), run(() -> runStatePeriodic(SuperstructureState.ZERO_ELEVATOR))),
                        runOnce(() -> elevatorZeroed = true))
                .finallyDo(() -> setState(SuperstructureState.STOW))
                .withName("SuperstructureZeroElevator");
    }

    public void runElevatorCharacterization(Current current) {
        elevator.runCharacterization(current);
    }

    public void runArmCharacterization(Voltage voltage) {
        arm.runCharacterization(voltage);
    }
}

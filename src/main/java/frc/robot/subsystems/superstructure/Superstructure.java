package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class Superstructure extends SubsystemBase {
    private static final double armAngleOffsetDegrees = 90 - ElevatorConstants.elevatorAngle.in(Degrees);

    private final Elevator elevator;
    private final Arm arm;
    private final Rollers rollers;
    private final RobotState robotState;
    private final SuperstructureController controller = new GraphController(this);

    private final @AutoLogOutput LoggedMechanism2d mechanism;

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;

    private @AutoLogOutput @Getter SuperstructureState state = SuperstructureState.START;

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
        this.state = state;

        rollers.setState(state.getData().getRollerState());

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

    public Command runState(SuperstructureState state) {
        switch (state) {
            case START:
                Commands.print("START is not intended to be run")
                        .andThen(hold())
                        .withName("SuperstructureStart");
            case STOP:
                return stop();
            case HOLD:
                return hold();
            case RETRACT_ARM:
                return retractArm();
            case ZERO_ELEVATOR:
                return zeroElevator();
            default:
                return defer(() -> controller.getCommand(this.state, state).withName("SuperstructureRunState"));
        }
    }

    public Command hold() {
        return runOnce(() -> setState(SuperstructureState.HOLD)).withName("SuperstructureHold");
    }

    public Command stop() {
        return runOnce(() -> setState(SuperstructureState.STOP)).withName("SuperstructureStop");
    }

    public Command retractArm() {
        return runOnce(() -> setState(SuperstructureState.RETRACT_ARM)).withName("SuperstructureRetractArm");
    }

    public Command zeroElevator() {
        return runOnce(() -> setState(SuperstructureState.ZERO_ELEVATOR))
                .andThen(elevator.zero())
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

package frc.robot.commands.characterization;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SuperstructureCharacterization {
    public static SysIdCommandSet elevatorSysId(Superstructure superstructure) {
        var maxElevatorHeight = ElevatorConstants.maximumHeight.times(0.8);
        var minElevatorHeight = ElevatorConstants.maximumHeight.times(0.2);

        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(5).per(Second),
                        Volts.of(60),
                        null,
                        state -> Logger.recordOutput("SysId/Elevator", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> superstructure.runElevatorCharacterization(Amps.of(output.in(Volts))),
                        null,
                        superstructure));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .finishedForward(() -> superstructure.getElevatorHeight().gt(maxElevatorHeight))
                .finishedReverse(() -> superstructure.getElevatorHeight().lt(minElevatorHeight))
                .beforeStarting(() -> superstructure.setState(SuperstructureState.HOLD))
                .finallyDo(() -> superstructure.setState(SuperstructureState.HOLD))
                .build();
    }

    public static SysIdCommandSet armSysId(Superstructure superstructure) {
        var maxArmAngle = Degrees.of(45);
        var minArmAngle = Degrees.of(0);

        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(4), null, state -> Logger.recordOutput("SysId/Arm", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> superstructure.runArmCharacterization(output), null, superstructure));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .finishedForward(() -> superstructure.getArmAngle().gt(maxArmAngle))
                .finishedReverse(() -> superstructure.getArmAngle().lt(minArmAngle))
                .beforeStarting(() -> superstructure.setState(SuperstructureState.HOLD))
                .finallyDo(() -> superstructure.setState(SuperstructureState.HOLD))
                .build();
    }
}

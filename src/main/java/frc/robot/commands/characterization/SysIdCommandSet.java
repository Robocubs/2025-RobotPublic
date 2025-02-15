package frc.robot.commands.characterization;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Builder;
import lombok.Builder.Default;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@Builder
public class SysIdCommandSet {
    private final SysIdRoutine sysIdRoutine;
    private final @Default BooleanSupplier finishedForward = () -> false;
    private final @Default BooleanSupplier finishedReverse = () -> false;
    private final @Default Runnable beforeStarting = () -> {};
    private final @Default Runnable finallyDo = () -> {};

    public void addAutos(AutoChooser autoChooser, String name) {
        autoChooser.addCmd(name + " Quasistatic Forward", () -> sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(finishedForward)
                .beforeStarting(beforeStarting)
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Quasistatic Reverse", () -> sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kReverse)
                .until(finishedReverse)
                .beforeStarting(beforeStarting)
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Dynamic Forward", () -> sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kForward)
                .until(finishedForward)
                .beforeStarting(beforeStarting)
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Dynamic Reverse", () -> sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kReverse)
                .until(finishedReverse)
                .beforeStarting(beforeStarting)
                .finallyDo(finallyDo));
    }
}

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SubsystemScheduler<Subsystem extends SubsystemBase> extends Command {
    private final Subsystem subsystem;
    private Command defaultCommand;
    private Command command;
    private Command nextCommand = null;
    private int loop = 0;

    public SubsystemScheduler(Subsystem subsystem, Command defaultCommand) {
        this.subsystem = subsystem;

        CommandScheduler.getInstance().registerComposedCommands(defaultCommand);
        this.defaultCommand = defaultCommand;
        command = this.defaultCommand;

        addRequirements(subsystem);
    }

    @Override
    public final void initialize() {
        command = this.defaultCommand;
        command.initialize();
    }

    @Override
    public final void execute() {
        Logger.recordOutput("SubsystemScheduler/Loops", loop++);
        Logger.recordOutput("SubsystemScheduler/Command", command.getName());

        if (nextCommand != null) {
            command.end(true);
            command = nextCommand;
            command.initialize();
            nextCommand = null;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(isFinished());
            command = defaultCommand;
            command.initialize();
        }
    }

    @Override
    public final void end(boolean interrupted) {
        command.end(interrupted);
    }

    public final Command schedule(Function<Subsystem, Command> command) {
        var commandValue = command.apply(subsystem);
        CommandScheduler.getInstance().registerComposedCommands(commandValue);
        return Commands.runOnce(() -> {
            System.out.println("Scheduling command " + commandValue.getName());
            nextCommand = commandValue;
        });
    }
}

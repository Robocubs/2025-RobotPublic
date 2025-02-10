package frc.robot.subsystems.superstructure.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.SuperstructureState;

public interface SuperstructureController {
    public Command getCommand(SuperstructureState state);
}

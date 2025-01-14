package frc.robot.subsystems.superstructure.arm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class Arm {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Optional<Rotation2d> holdAngle = Optional.empty();

    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public enum Goal {
        STOP(),
        HOLD(),
        STOW(Rotation2d.fromDegrees(80)),
        L1_SCORE_LONG(Rotation2d.fromDegrees(180)),
        L1_SCORE_WIDE(Rotation2d.fromDegrees(110)),
        L23_SCORE(Rotation2d.fromDegrees(90)),
        L23_INTAKE(Rotation2d.fromDegrees(110)),
        L4_SCORE(Rotation2d.fromDegrees(110)),
        BARGE_SCORE(Rotation2d.fromDegrees(110)),
        ALGAE_INTAKE(Rotation2d.fromDegrees(150)),
        CORAL_INTAKE_FLOOR(Rotation2d.fromDegrees(240)),
        CORAL_INTAKE_FUNNEL(Rotation2d.fromDegrees(45)),
        PROCESSOR(Rotation2d.fromDegrees(90));

        private final Rotation2d angle;

        private Goal() {
            this.angle = Rotation2d.kZero;
        }

        private Goal(Rotation2d angle) {
            this.angle = angle;
        }
    }

    public Arm(ArmIO io) {
        this.io = io;
        Logger.processInputs("Arm", inputs);
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setGoal(Goal goal) {
        /*
         * TODO: Control the arm to rotate to the specified goal
         * 1. Set the goal to the specified goal
         * 2. Set the position on the IO
         *
         * If the goal is HOLD, set the angle to the current measured value.
         * Make sure that this angle does not change if the goal is already HOLD.
         */
    }

    public void stop() {
        // TODO: Stop the arm from moving
        // Set the goal to stop
    }

    @AutoLogOutput
    public boolean atGoal() {
        // TODO: Return whether the arm is at the goal within a tolerance set in ArmConstants
        return false;
    }
}

package frc.robot;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final boolean tuningEnabled = false;
    private static final Mode simMode = Mode.SIM;
    private static final RobotType realBot = RobotType.COMP_BOT;
    private static final RobotType simBot = RobotType.SIM_BOT;

    public static final Mode mode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final RobotType robot = (mode == Mode.REAL || mode == Mode.REPLAY) ? realBot : simBot;

    public static enum RobotType {
        COMP_BOT,
        SIM_BOT
    }

    public static enum Mode {
        REAL,
        REPLAY,
        SIM
    }

    public static final Time mainLoopPeriod = Milliseconds.of(20);

    public static final Mass coralMass = Pounds.of((1.1 + 1.8) / 2);
    public static final Mass algaeMass = Pounds.of(1.5);
}

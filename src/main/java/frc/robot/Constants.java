package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
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
}

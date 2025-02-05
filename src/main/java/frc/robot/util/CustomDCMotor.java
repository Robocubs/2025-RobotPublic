package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class CustomDCMotor {
    public static DCMotor getKrakenX44(int numMotors) {
        return new DCMotor(12, 4.05, 275.0, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), numMotors);
    }
}

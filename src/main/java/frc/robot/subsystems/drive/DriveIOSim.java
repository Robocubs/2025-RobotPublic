package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class DriveIOSim extends DriveIOHardware {
    private static final double kSimLoopPeriod = 0.005;
    private final Notifier simNotifier;
    private double lastSimTime;

    public DriveIOSim() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at a faster rate so PID gains behave more reasonably
        simNotifier = new Notifier(() -> {
            final var currentTime = Utils.getCurrentTimeSeconds();
            var deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            // Use the measured time delta, get battery voltage from WPILib
            swerveDrivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}

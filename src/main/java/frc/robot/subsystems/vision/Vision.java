package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.apriltag.AprilTagCamera;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;

public class Vision extends SubsystemBase {
    private final AprilTagCamera[] cameras;
    private final RobotState robotState;

    public Vision(AprilTagIO[] tagIO, RobotState robotState) {
        this.robotState = robotState;
        this.cameras =
                Stream.of(tagIO).map(io -> new AprilTagCamera(io, robotState)).toArray(AprilTagCamera[]::new);
    }

    @Override
    public void periodic() {
        List<VisionMeasurement> measurements = new ArrayList<>(cameras.length * 4);
        for (var camera : cameras) {
            camera.update(measurements);
        }

        robotState.addVisionMeasurements(measurements.toArray(VisionMeasurement[]::new));
    }
}

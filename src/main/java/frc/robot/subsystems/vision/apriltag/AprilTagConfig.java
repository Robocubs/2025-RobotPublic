package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagConfig {
    public final String cameraName;
    public final Transform3d robotToCamera;
    public final Transform3d cameraToRobot;
    public final double weight;

    public AprilTagConfig(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.cameraToRobot = robotToCamera.inverse();
        this.weight = 1.0;
    }
}

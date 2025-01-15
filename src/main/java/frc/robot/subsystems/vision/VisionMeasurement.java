package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {
    public final double timestampSeconds;
    public final Pose3d pose;
    public final Matrix<N3, N1> stdDevs;

    public VisionMeasurement(double timestamp, Pose3d pose, Matrix<N3, N1> stdDevs) {
        this.timestampSeconds = timestamp;
        this.pose = pose;
        this.stdDevs = stdDevs;
    }
}

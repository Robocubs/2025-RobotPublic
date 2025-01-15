package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
    @AutoLog
    public class AprilTagIOInputs {
        public boolean isConnected;
        public AprilTagPipelineResults pipelineResults = new AprilTagPipelineResults();
        public int fps;
        public int temperature;
        public int latency;
    }

    public default void updateInputs(AprilTagIOInputs inputs) {}

    public AprilTagConfig getConfig();
}

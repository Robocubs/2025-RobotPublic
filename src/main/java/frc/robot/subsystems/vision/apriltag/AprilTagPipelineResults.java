package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.AprilTagPipelineResultsProto;

public class AprilTagPipelineResults implements ProtobufSerializable {
    public final AprilTagPipelineResult[] results;

    public AprilTagPipelineResults() {
        results = new AprilTagPipelineResult[] {};
    }

    public AprilTagPipelineResults(AprilTagPipelineResult[] results) {
        this.results = results;
    }

    public static final AprilTagPipelineResultsProto proto = new AprilTagPipelineResultsProto();
}

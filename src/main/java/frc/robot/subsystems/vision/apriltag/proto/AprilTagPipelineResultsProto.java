package frc.robot.subsystems.vision.apriltag.proto;

import com.robocubs.proto.Apriltag.ProtobufAprilTagPipelineResults;
import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.subsystems.vision.apriltag.AprilTagPipelineResult;
import frc.robot.subsystems.vision.apriltag.AprilTagPipelineResults;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class AprilTagPipelineResultsProto
        implements Protobuf<AprilTagPipelineResults, ProtobufAprilTagPipelineResults> {

    @Override
    public Class<AprilTagPipelineResults> getTypeClass() {
        return AprilTagPipelineResults.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagPipelineResults.getDescriptor();
    }

    @Override
    public ProtobufAprilTagPipelineResults createMessage() {
        return ProtobufAprilTagPipelineResults.newInstance();
    }

    @Override
    public AprilTagPipelineResults unpack(ProtobufAprilTagPipelineResults msg) {
        return new AprilTagPipelineResults(AprilTagPipelineResult.proto.unpack(msg.getResults()));
    }

    @Override
    public void pack(ProtobufAprilTagPipelineResults msg, AprilTagPipelineResults value) {
        AprilTagPipelineResult.proto.pack(msg.getMutableResults(), value.results);
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}

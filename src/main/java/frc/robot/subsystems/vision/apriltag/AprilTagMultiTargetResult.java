package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.AprilTagMultiTargetResultProto;

public class AprilTagMultiTargetResult implements ProtobufSerializable {
    public final int[] targetIds;
    public final Pose3d cameraPose;
    public final double reprojectionError;

    public AprilTagMultiTargetResult(int[] targetIds, Pose3d cameraPose, double reprojectionError) {
        this.targetIds = targetIds;
        this.cameraPose = cameraPose;
        this.reprojectionError = reprojectionError;
    }

    public static final AprilTagMultiTargetResultProto proto = new AprilTagMultiTargetResultProto();
}

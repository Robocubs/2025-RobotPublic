package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.rollers.Rollers;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Builder.Default;
import lombok.Getter;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@AllArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@Builder(toBuilder = true)
@Getter
public class SuperstructureStateData {
    private final @Default Rollers.State rollerState = Rollers.State.HOLD;
    private final @Default SuperstructurePose pose = new SuperstructurePose();
    // private final @Default LinearVelocity maxRobotSpeed = DriveConstants.maxSpeed;

    // @SuppressWarnings("unused")
    // public static class SuperstructureStateDataBuilder {
    //     private SuperstructurePose pose;
    //     private LinearVelocity maxRobotSpeed;

    //     public SuperstructureStateDataBuilder pose(SuperstructurePose pose) {
    //         this.pose = pose;
    //         this.maxRobotSpeed = RobotState.getMaxRobotSpeed(pose.elevatorHeight());
    //         return this;
    //     }
    // }
}

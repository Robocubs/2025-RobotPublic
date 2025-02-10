package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.rollers.Rollers;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@AllArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@Builder(toBuilder = true)
@Getter
public class SuperstructureStateData {
    public static SuperstructureStateData none() {
        return new SuperstructureStateData();
    }

    @Builder.Default
    private final SuperstructurePose pose = new SuperstructurePose();

    @Builder.Default
    private final Rollers.State rollerState = Rollers.State.HOLD;
}

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
    private final @Default SuperstructurePose algaePose = null;

    public SuperstructurePose getAlgaePose() {
        return algaePose == null ? pose : algaePose;
    }
}

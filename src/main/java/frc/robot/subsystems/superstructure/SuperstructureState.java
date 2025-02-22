package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.subsystems.superstructure.rollers.Rollers;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
@Getter
public enum SuperstructureState {
    // Special states
    START(SuperstructureStateData.builder().rollerState(Rollers.State.HOLD).build()),
    STOP(SuperstructureStateData.builder().rollerState(Rollers.State.STOPPED).build()),
    HOLD(SuperstructureStateData.builder().rollerState(Rollers.State.HOLD).build()),
    RETRACT_ARM(SuperstructureStateData.builder()
            .pose(SuperstructurePose.Preset.STOW.getRetractPose())
            .build()),
    ZERO_ELEVATOR(SuperstructureStateData.builder()
            .pose(Preset.STOW.getPose())
            .rollerState(Rollers.State.HOLD)
            .build()),

    // Preset states
    STOW(SuperstructureStateData.builder().pose(Preset.STOW.getPose()).build()),
    FEED_RETRACTED(
            SuperstructureStateData.builder().pose(Preset.FEED.getRetractPose()).build()),
    FEED(SuperstructureStateData.builder()
            .pose(Preset.FEED.getPose())
            .rollerState(Rollers.State.AUTO_FEED_CORAL)
            .build()),
    L1_CORAL_WIDE(
            SuperstructureStateData.builder().pose(Preset.L1_WIDE.getPose()).build()),
    L1_CORAL_WIDE_SCORE(SuperstructureStateData.builder()
            .pose(Preset.L1_WIDE.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    L1_CORAL(SuperstructureStateData.builder().pose(Preset.L1_LONG.getPose()).build()),
    L1_CORAL_SCORE(SuperstructureStateData.builder()
            .pose(Preset.L1_LONG.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    L2_CORAL(SuperstructureStateData.builder().pose(Preset.L2.getPose()).build()),
    L2_CORAL_SCORE(SuperstructureStateData.builder()
            .pose(Preset.L2.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    L3_CORAL(SuperstructureStateData.builder().pose(Preset.L3.getPose()).build()),
    L3_CORAL_SCORE(SuperstructureStateData.builder()
            .pose(Preset.L3.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    L4_CORAL_RETRACTED(
            SuperstructureStateData.builder().pose(Preset.L4.getRetractPose()).build()),
    L4_CORAL(SuperstructureStateData.builder().pose(Preset.L4.getPose()).build()),
    L4_CORAL_SCORE(SuperstructureStateData.builder()
            .pose(Preset.L4.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    BARGE_RETRACTED(SuperstructureStateData.builder()
            .pose(Preset.BARGE.getRetractPose())
            .build()),
    BARGE(SuperstructureStateData.builder().pose(Preset.BARGE.getPose()).build()),
    BARGE_SCORE(SuperstructureStateData.builder()
            .pose(Preset.BARGE.getPose())
            .rollerState(Rollers.State.CORAL_FORWARD)
            .build()),
    L2_ALGAE_RETRACTED(SuperstructureStateData.builder()
            .pose(Preset.L2_ALGAE.getRetractPose())
            .build()),
    L2_ALGAE(SuperstructureStateData.builder()
            .pose(Preset.L2_ALGAE.getPose())
            .rollerState(Rollers.State.AUTO_INTAKE_ALGAE)
            .build()),
    L3_ALGAE_RETRACTED(SuperstructureStateData.builder()
            .pose(Preset.L3_ALGAE.getRetractPose())
            .build()),
    L3_ALGAE(SuperstructureStateData.builder()
            .pose(Preset.L3_ALGAE.getPose())
            .rollerState(Rollers.State.AUTO_INTAKE_ALGAE)
            .build()),
    ALGAE_INTAKE(SuperstructureStateData.builder()
            .pose(Preset.ALGAE_INTAKE.getPose())
            .rollerState(Rollers.State.AUTO_INTAKE_ALGAE)
            .build()),
    CORAL_INTAKE_1_RETRACTED(SuperstructureStateData.builder()
            .pose(Preset.CORAL_INTAKE_1.getRetractPose())
            .build()),
    CORAL_INTAKE_1(SuperstructureStateData.builder()
            .pose(Preset.CORAL_INTAKE_1.getPose())
            .rollerState(Rollers.State.AUTO_INTAKE_CORAL)
            .build()),
    CORAL_INTAKE_2(SuperstructureStateData.builder()
            .pose(Preset.CORAL_INTAKE_2.getPose())
            .rollerState(Rollers.State.AUTO_INTAKE_CORAL)
            .build()),
    PROCESSOR(SuperstructureStateData.builder().pose(Preset.PROCESSOR.getPose()).build()),
    PROCESSOR_SCORE(SuperstructureStateData.builder()
            .pose(Preset.PROCESSOR.getPose())
            .rollerState(Rollers.State.ALGAE_FORWARD)
            .build());

    private final SuperstructureStateData data;
}

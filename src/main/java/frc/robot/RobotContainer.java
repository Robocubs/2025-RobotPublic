package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.AlgaeMode;
import frc.robot.RobotState.ReefMode;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.characterization.DriveCharacterization;
import frc.robot.commands.characterization.SuperstructureCharacterization;
import frc.robot.controls.StreamDeck;
import frc.robot.controls.StreamDeck.StreamDeckButton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.rollers.RollersIO;
import frc.robot.subsystems.superstructure.rollers.RollersIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOSim;
import frc.robot.util.TriggeredAlert;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final RobotState RobotState = new RobotState();
    private final StreamDeck streamDeck = new StreamDeck();
    private final AutoChooser autoChooser = new AutoChooser();

    // Subsystems
    private final RobotState robotState;
    private final Drive drive;
    private final Superstructure superstructure;

    // State triggers
    private final Trigger teleop = RobotModeTriggers.teleop();
    private final Trigger disabled = RobotModeTriggers.disabled();

    public RobotContainer() {
        Drive drive = null;
        Vision vision = null;
        Superstructure superstructure = null;

        this.robotState = new RobotState();

        if (Constants.mode != Mode.REPLAY) {
            switch (Constants.robot) {
                case COMP_BOT:
                    drive = new Drive(new DriveIOHardware(), robotState);
                    break;
                case SIM_BOT:
                    drive = new Drive(new DriveIOSim(), robotState);
                    vision = new Vision(
                            new AprilTagIO[] {
                                new AprilTagIOSim(VisionConstants.frontAprilTagConfig, robotState),
                                new AprilTagIOSim(VisionConstants.backAprilTagConfig, robotState)
                            },
                            robotState);
                    superstructure =
                            new Superstructure(new ElevatorIOSim(), new ArmIOSim(), new RollersIOSim(), robotState);
                    break;
            }
        }

        if (drive == null) {
            drive = new Drive(new DriveIO() {}, robotState);
        }

        if (superstructure == null) {
            superstructure = new Superstructure(new ElevatorIO() {}, new ArmIO() {}, new RollersIO() {}, robotState);
        }

        if (vision == null) {
            vision = new Vision(new AprilTagIO[] {}, robotState);
        }

        this.drive = drive;
        this.superstructure = superstructure;

        configureBindings();
        configureAutoRoutines();
    }

    private void configureBindings() {
        new TriggeredAlert(
                "Driver controller disconnected",
                AlertType.kError,
                () -> !DriverStation.isJoystickConnected(
                                driverController.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(
                                driverController.getHID().getPort()));
        DriverStation.silenceJoystickConnectionWarning(true);

        drive.setDefaultCommand(drive.withJoysticksEnhanced(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        teleop.onTrue(drive.resetRotation(robotState::getHeading));

        driverController.start().onTrue(drive.resetRotation());
        driverController.x().whileTrue(drive.brake());
        driverController.back().onTrue(superstructure.zeroElevator());

        driverController
                .rightTrigger()
                .and(() -> RobotState.isSelected(ReefMode.L4_CORAL))
                .whileTrue(superstructure.score(
                        SuperstructureState.L4_CORAL,
                        SuperstructureState.L4_CORAL_SCORE,
                        driverController.rightBumper()));
        driverController
                .rightTrigger()
                .and(() -> RobotState.isSelected(ReefMode.L3_CORAL))
                .whileTrue(superstructure.score(
                        SuperstructureState.L3_CORAL,
                        SuperstructureState.L3_CORAL_SCORE,
                        driverController.rightBumper()));
        driverController
                .rightTrigger()
                .and(() -> RobotState.isSelected(ReefMode.L2_CORAL))
                .whileTrue(superstructure.score(
                        SuperstructureState.L2_CORAL,
                        SuperstructureState.L2_CORAL_SCORE,
                        driverController.rightBumper()));
        driverController
                .rightTrigger()
                .and(() -> RobotState.isSelected(ReefMode.L1_CORAL))
                .whileTrue(superstructure.score(
                        SuperstructureState.L1_CORAL,
                        SuperstructureState.L1_CORAL_SCORE,
                        driverController.rightBumper()));

        driverController
                .leftTrigger()
                .and(() -> RobotState.isSelected(AlgaeMode.PROCESSOR))
                .whileTrue(superstructure.runState(SuperstructureState.PROCESSOR_SCORE));

        driverController
                .leftTrigger()
                .and(() -> RobotState.isSelected(AlgaeMode.L3))
                .whileTrue(superstructure.runState(SuperstructureState.L3_ALGAE));

        driverController
                .leftTrigger()
                .and(() -> RobotState.isSelected(AlgaeMode.L2))
                .whileTrue(superstructure.runState(SuperstructureState.L2_ALGAE));

        driverController
                .leftTrigger()
                .and(() -> RobotState.isSelected(AlgaeMode.BARGE))
                .whileTrue(superstructure.score(
                        SuperstructureState.BARGE, SuperstructureState.BARGE_SCORE, driverController.rightBumper()));

        /* Stream Dech Buttons */
        var l4CoralScore = RobotState.setReefSelection(ReefMode.L4_CORAL);
        var l3CoralScore = RobotState.setReefSelection(ReefMode.L3_CORAL);
        var l2CoralScore = RobotState.setReefSelection(ReefMode.L2_CORAL);
        var l1CoralScore = RobotState.setReefSelection(ReefMode.L1_CORAL);
        var l3Algae = RobotState.setAlgaeSelection(AlgaeMode.L3);
        var l2Algae = RobotState.setAlgaeSelection(AlgaeMode.L2);
        var coralIntake = superstructure.runState(SuperstructureState.CORAL_INTAKE_1);
        var algaeIntake = superstructure.runState(SuperstructureState.ALGAE_INTAKE);
        var feed = superstructure.runState(SuperstructureState.FEED);
        var stow = superstructure.runState(SuperstructureState.STOW);
        var processor = RobotState.setAlgaeSelection(AlgaeMode.PROCESSOR);
        var barge = RobotState.setAlgaeSelection(AlgaeMode.BARGE);

        // TODO: Make automatically switch between feed and stow when appropriate
        superstructure.setDefaultCommand(stow);
        teleop.onTrue(superstructure.hold(true));
        disabled.onTrue(superstructure.stop().ignoringDisable(true));

        streamDeck.configureButtons(
                config -> config.add(StreamDeckButton.L4_CORAL, () -> RobotState.isSelected(ReefMode.L4_CORAL))
                        .add(StreamDeckButton.L3_CORAL, () -> RobotState.isSelected(ReefMode.L3_CORAL))
                        .add(StreamDeckButton.L2_CORAL, () -> RobotState.isSelected(ReefMode.L2_CORAL))
                        .add(StreamDeckButton.L1_CORAL, () -> RobotState.isSelected(ReefMode.L1_CORAL))
                        .add(StreamDeckButton.L3_ALGAE, () -> RobotState.isSelected(AlgaeMode.L3))
                        .add(StreamDeckButton.L2_ALGAE, () -> RobotState.isSelected(AlgaeMode.L2))
                        .add(StreamDeckButton.CORAL_INTAKE, () -> coralIntake.isScheduled())
                        .add(StreamDeckButton.ALGAE_INTAKE, () -> algaeIntake.isScheduled())
                        .add(StreamDeckButton.FEED, () -> feed.isScheduled())
                        .add(StreamDeckButton.STOW, () -> stow.isScheduled())
                        .add(StreamDeckButton.PROCESSOR, () -> RobotState.isSelected(AlgaeMode.PROCESSOR))
                        .add(StreamDeckButton.BARGE, () -> RobotState.isSelected(AlgaeMode.BARGE)));

        streamDeck.button(StreamDeckButton.L4_CORAL).onTrue(l4CoralScore);
        streamDeck.button(StreamDeckButton.L3_CORAL).onTrue(l3CoralScore);
        streamDeck.button(StreamDeckButton.L2_CORAL).onTrue(l2CoralScore);
        streamDeck.button(StreamDeckButton.L1_CORAL).onTrue(l1CoralScore);
        streamDeck.button(StreamDeckButton.L3_ALGAE).onTrue(l3Algae);
        streamDeck.button(StreamDeckButton.L2_ALGAE).onTrue(l2Algae);
        streamDeck.button(StreamDeckButton.CORAL_INTAKE).onTrue(coralIntake);
        streamDeck.button(StreamDeckButton.ALGAE_INTAKE).onTrue(algaeIntake);
        streamDeck.button(StreamDeckButton.FEED).onTrue(feed);
        streamDeck.button(StreamDeckButton.STOW).onTrue(stow);
        streamDeck.button(StreamDeckButton.PROCESSOR).onTrue(processor);
        streamDeck.button(StreamDeckButton.BARGE).onTrue(barge);
    }

    private void configureAutoRoutines() {
        var autoRoutines = new AutoRoutines(robotState, drive);

        autoChooser.addRoutine("Demo", autoRoutines::demoAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.select("Demo");

        if (Constants.characterizationEnabled) {
            autoChooser.addCmd("Drive Wheel Characterization", () -> DriveCharacterization.wheelRadius(drive));
            DriveCharacterization.translationSysId(drive).addAutos(autoChooser, "Drive Translation");
            DriveCharacterization.steerSysId(drive).addAutos(autoChooser, "Drive Steer");
            DriveCharacterization.rotationSysId(drive).addAutos(autoChooser, "Drive Rotation");
            SuperstructureCharacterization.elevatorSysId(superstructure).addAutos(autoChooser, "Elevator");
            SuperstructureCharacterization.armSysId(superstructure).addAutos(autoChooser, "Arm");
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
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

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final StreamDeck StreamDeck = new StreamDeck();
    private final choreo.auto.AutoChooser autoChooser = new AutoChooser();

    // Subsystems
    private final RobotState robotState;
    private final Drive drive;
    private final Superstructure superstructure;

    // State triggers
    private final Trigger teleop = RobotModeTriggers.teleop();

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

        drive.setDefaultCommand(drive.withJoysticks(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        teleop.onTrue(drive.resetRotation(robotState::getHeading));

        driverController.start().onTrue(drive.resetRotation());
        driverController.a().whileTrue(drive.brake());
        driverController
                .b()
                .whileTrue(drive.pointModules(
                        () -> new Rotation2d(driverController.getLeftY(), driverController.getLeftX())));
        driverController.x().whileTrue(drive.toPose(() -> new Pose2d(8.33, 4, Rotation2d.kCCW_90deg), false));

        driverController.leftBumper().onTrue(superstructure.runState(SuperstructureState.BARGE));
        driverController.rightBumper().onTrue(superstructure.runState(SuperstructureState.FEED));
        driverController.povUp().onTrue(superstructure.runState(SuperstructureState.L4_CORAL_SCORE));
        driverController.povLeft().onTrue(superstructure.runState(SuperstructureState.L3_CORAL_SCORE));
        driverController.povRight().onTrue(superstructure.runState(SuperstructureState.L2_CORAL_SCORE));
        driverController.povDown().onTrue(superstructure.runState(SuperstructureState.CORAL_INTAKE_2));
        driverController.back().onTrue(superstructure.zeroElevator());

        /* Stream Dech Buttons */
        var l4CoralScore =
                superstructure.runState(SuperstructureState.L4_CORAL_SCORE).withName("l4Coral");
        var l3CoralScore =
                superstructure.runState(SuperstructureState.L3_CORAL_SCORE).withName("l3Coral");
        var l2CoralScore =
                superstructure.runState(SuperstructureState.L2_CORAL_SCORE).withName("l2Coral");
        var l1CoralScore =
                superstructure.runState(SuperstructureState.L1_CORAL_SCORE).withName("l1Coral");
        var l1CoralWideScore =
                superstructure.runState(SuperstructureState.L1_CORAL_WIDE_SCORE).withName("l1CoralWide");
        var coralIntake1 =
                superstructure.runState(SuperstructureState.CORAL_INTAKE_1).withName("coralIntake1");
        var coralIntake2 =
                superstructure.runState(SuperstructureState.CORAL_INTAKE_2).withName("coralIntake2");
        var feed = superstructure.runState(SuperstructureState.FEED).withName("feed");
        var stow = superstructure.runState(SuperstructureState.STOW).withName("stow");
        var barge = superstructure.runState(SuperstructureState.BARGE).withName("barge");
        var l3AlgaeIntake =
                superstructure.runState(SuperstructureState.L3_ALGAE).withName("l3AlgaeIntake");
        var l2AlgaeIntake =
                superstructure.runState(SuperstructureState.L2_ALGAE).withName("l2AlgaeIntake");

        StreamDeck.configureButton(config -> config.add(
                        StreamDeckButton.l4ScoreButton,
                        () -> superstructure.getState() == SuperstructureState.L4_CORAL_SCORE)
                .add(
                        StreamDeckButton.l3ScoreButton,
                        () -> superstructure.getState() == SuperstructureState.L3_CORAL_SCORE)
                .add(
                        StreamDeckButton.l2ScoreButton,
                        () -> superstructure.getState() == SuperstructureState.L2_CORAL_SCORE)
                .add(
                        StreamDeckButton.l1ScoreButton,
                        () -> superstructure.getState() == SuperstructureState.L1_CORAL_SCORE)
                .add(
                        StreamDeckButton.l1WideScoreButton,
                        () -> superstructure.getState() == SuperstructureState.L1_CORAL_WIDE_SCORE)
                .add(
                        StreamDeckButton.coralIntake1Button,
                        () -> superstructure.getState() == SuperstructureState.CORAL_INTAKE_1)
                .add(
                        StreamDeckButton.coralIntake2Button,
                        () -> superstructure.getState() == SuperstructureState.CORAL_INTAKE_2)
                .add(StreamDeckButton.feedButton, () -> superstructure.getState() == SuperstructureState.FEED)
                .add(StreamDeckButton.stowButton, () -> superstructure.getState() == SuperstructureState.STOW)
                .add(StreamDeckButton.bargeButton, () -> superstructure.getState() == SuperstructureState.BARGE)
                .add(
                        StreamDeckButton.l3AlgaeIntakeButton,
                        () -> superstructure.getState() == SuperstructureState.L3_ALGAE)
                .add(
                        StreamDeckButton.l2AlgaeIntakeButton,
                        () -> superstructure.getState() == SuperstructureState.L2_ALGAE));

        StreamDeck.button(StreamDeckButton.l4ScoreButton).onTrue(l4CoralScore);
        StreamDeck.button(StreamDeckButton.l3ScoreButton).onTrue(l3CoralScore);
        StreamDeck.button(StreamDeckButton.l2ScoreButton).onTrue(l2CoralScore);
        StreamDeck.button(StreamDeckButton.l1ScoreButton).onTrue(l1CoralScore);
        StreamDeck.button(StreamDeckButton.l1WideScoreButton).onTrue(l1CoralWideScore);
        StreamDeck.button(StreamDeckButton.coralIntake1Button).onTrue(coralIntake1);
        StreamDeck.button(StreamDeckButton.coralIntake2Button).onTrue(coralIntake2);
        StreamDeck.button(StreamDeckButton.feedButton).onTrue(feed);
        StreamDeck.button(StreamDeckButton.stowButton).onTrue(stow);
        StreamDeck.button(StreamDeckButton.bargeButton).onTrue(barge);
        StreamDeck.button(StreamDeckButton.l3AlgaeIntakeButton).onTrue(l3AlgaeIntake);
        StreamDeck.button(StreamDeckButton.l2AlgaeIntakeButton).onTrue(l2AlgaeIntake);
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

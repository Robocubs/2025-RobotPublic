package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.AlgaeMode;
import frc.robot.RobotState.CoralMode;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.AutoIntakeAlgae;
import frc.robot.commands.AutoScore;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.commands.characterization.DriveCharacterization;
import frc.robot.commands.characterization.SuperstructureCharacterization;
import frc.robot.controls.StreamDeck;
import frc.robot.controls.StreamDeck.StreamDeckButton;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOHardware;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIOHardware;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOHardware;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOHardware;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.funnel.FunnelIO;
import frc.robot.subsystems.superstructure.funnel.FunnelIOHardware;
import frc.robot.subsystems.superstructure.funnel.FunnelIOSim;
import frc.robot.subsystems.superstructure.rollers.RollersIO;
import frc.robot.subsystems.superstructure.rollers.RollersIOHardware;
import frc.robot.subsystems.superstructure.rollers.RollersIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOHardware;
import frc.robot.subsystems.vision.apriltag.AprilTagIOSim;
import frc.robot.util.TriggeredAlert;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final StreamDeck streamDeck = new StreamDeck();
    private final AutoChooser autoChooser = new AutoChooser();

    // Subsystems
    private final RobotState robotState = new RobotState();
    private final Drive drive;
    private final Superstructure superstructure;
    private final Climb climb;

    // State triggers
    private final Trigger teleop = RobotModeTriggers.teleop();
    private final Trigger disabled = RobotModeTriggers.disabled();

    // On-robot buttons
    private final DigitalInput coastButtonInput = new DigitalInput(0);

    public RobotContainer() {
        Drive drive = null;
        Vision vision = null;
        Superstructure superstructure = null;
        Climb climb = null;
        LED led = null;

        if (Constants.mode != Mode.REPLAY) {
            switch (Constants.robot) {
                case COMP_BOT:
                    drive = new Drive(new DriveIOHardware(), robotState);
                    vision = new Vision(
                            new AprilTagIO[] {
                                new AprilTagIOHardware(VisionConstants.frontLeftAprilTagConfig),
                                new AprilTagIOHardware(VisionConstants.frontRightAprilTagConfig)
                            },
                            robotState);
                    superstructure = new Superstructure(
                            new ElevatorIOHardware(),
                            new ArmIOHardware(),
                            new RollersIOHardware(),
                            new FunnelIOHardware(),
                            robotState);
                    climb = new Climb(new ClimbIOHardware(), robotState);
                    led = new LED(new LEDIOHardware(), robotState);
                    break;
                case SIM_BOT:
                    var simState = new SimState();

                    drive = new Drive(new DriveIOSim(), robotState);
                    vision = new Vision(
                            new AprilTagIO[] {
                                new AprilTagIOSim(VisionConstants.frontLeftAprilTagConfig, robotState),
                                new AprilTagIOSim(VisionConstants.frontRightAprilTagConfig, robotState)
                            },
                            robotState);
                    superstructure = new Superstructure(
                            new ElevatorIOSim(),
                            new ArmIOSim(),
                            new RollersIOSim(simState),
                            new FunnelIOSim(),
                            robotState);
                    climb = new Climb(new ClimbIOSim(), robotState);
                    break;
            }
        }

        if (drive == null) {
            drive = new Drive(new DriveIO() {}, robotState);
        }

        if (superstructure == null) {
            superstructure = new Superstructure(
                    new ElevatorIO() {}, new ArmIO() {}, new RollersIO() {}, new FunnelIO() {}, robotState);
        }

        if (vision == null) {
            vision = new Vision(
                    new AprilTagIO[] {
                        () -> VisionConstants.frontLeftAprilTagConfig, () -> VisionConstants.frontRightAprilTagConfig
                    },
                    robotState);
        }

        if (climb == null) {
            climb = new Climb(new ClimbIO() {}, robotState);
        }

        if (led == null) {
            led = new LED(new LEDIO() {}, robotState);
        }

        this.drive = drive;
        this.superstructure = superstructure;
        this.climb = climb;

        SmartDashboard.putData("Drive", drive);
        SmartDashboard.putData("Superstructure", superstructure);
        SmartDashboard.putData("Climb", climb);

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
                () -> -driverController.getRightX(),
                driverController.leftBumper(),
                driverController.leftTrigger(),
                driverController.rightTrigger(),
                driverController.a()));

        /* Driver Controller */
        driverController.start().onTrue(drive.resetRotation());
        driverController.x().whileTrue(drive.brake());
        driverController.back().onTrue(superstructure.zeroElevator());

        driverController
                .leftTrigger()
                .and(() -> robotState.isSelected(CoralMode.L4_CORAL))
                .whileTrue(AutoScore.autoScore(drive, superstructure, robotState, robotState::getLeftReefPose));
        driverController
                .rightTrigger()
                .and(() -> robotState.isSelected(CoralMode.L4_CORAL))
                .whileTrue(AutoScore.autoScore(drive, superstructure, robotState, robotState::getRightReefPose));

        driverController
                .leftTrigger()
                .and(() -> robotState.isSelected(CoralMode.L2_CORAL) || robotState.isSelected(CoralMode.L3_CORAL))
                .whileTrue(AutoScore.autoScore(drive, superstructure, robotState, robotState::getLeftReefPose));
        driverController
                .rightTrigger()
                .and(() -> robotState.isSelected(CoralMode.L2_CORAL) || robotState.isSelected(CoralMode.L3_CORAL))
                .whileTrue(AutoScore.autoScore(drive, superstructure, robotState, robotState::getRightReefPose));

        driverController
                .rightTrigger()
                .and(() -> robotState.isSelected(CoralMode.L1_CORAL))
                .whileTrue(superstructure.runState(SuperstructureState.L1_CORAL_SCORE));
        driverController
                .rightBumper()
                .and(() -> robotState.isSelected(AlgaeMode.PROCESSOR))
                .whileTrue(superstructure.runState(SuperstructureState.PROCESSOR_SCORE));
        driverController // Score Auto Algae
                .a()
                .and(() -> robotState.isSelected(AlgaeMode.L3) || robotState.isSelected(AlgaeMode.L2))
                .whileTrue(
                        AutoIntakeAlgae.autoIntakeAlgae(drive, superstructure, robotState, () -> robotState.getPose()));
        driverController // Score High Algae
                .y()
                .and(() -> robotState.isSelected(AlgaeMode.L3) || robotState.isSelected(AlgaeMode.L2))
                .whileTrue(
                        AutoIntakeAlgae.intakeHighAlgae(drive, superstructure, robotState, () -> robotState.getPose()));
        driverController // Score Low Algae
                .b()
                .and(() -> robotState.isSelected(AlgaeMode.L3) || robotState.isSelected(AlgaeMode.L2))
                .whileTrue(
                        AutoIntakeAlgae.intakeLowAlgae(drive, superstructure, robotState, () -> robotState.getPose()));
        driverController
                .a()
                .and(() -> robotState.isSelected(AlgaeMode.BARGE))
                .whileTrue(superstructure
                        .score(
                                SuperstructureState.BARGE,
                                SuperstructureState.BARGE_SCORE,
                                driverController.rightBumper())
                        .unless(robotState::underNetArea));

        driverController
                .leftBumper()
                .and(() -> robotState.inBargeArea())
                .whileTrue(drive.fineTuneClimb(() -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

        /* Stream Deck Buttons */
        var stow = superstructure.runState(SuperstructureState.STOW);
        var feed = superstructure.runState(SuperstructureState.FEED);
        var l4CoralScore = robotState.setCoralSelection(CoralMode.L4_CORAL);
        var l3CoralScore = robotState.setCoralSelection(CoralMode.L3_CORAL);
        var l2CoralScore = robotState.setCoralSelection(CoralMode.L2_CORAL);
        var l1CoralScore = robotState.setCoralSelection(CoralMode.L1_CORAL);
        var algaeIntake = superstructure
                .runState(SuperstructureState.ALGAE_INTAKE)
                .until(robotState::hasAlgae)
                .andThen(() -> stow.schedule())
                .withName("SuperstructureRun_ALGAE_INTAKE");
        var algaeEject = superstructure.runState(SuperstructureState.ALGAE_EJECT);
        var bumpForwards = superstructure.bumpFeedPosition(Inches.of(1));
        var bumpReverse = superstructure.bumpFeedPosition(Inches.of(-1));
        var coralIntake = superstructure
                .runState(SuperstructureState.CORAL_INTAKE)
                .until(robotState::hasCoralLoaded)
                .andThen(() -> stow.schedule())
                .withName("SuperstructureRun_CORAL_INTAKE");
        var coralReject = superstructure.runState(SuperstructureState.HOLD);

        var climbDeploy = climb.deploy();
        var climbRetract = climb.retract();
        var climbZero = climb.zero();

        // TODO: Make automatically switch between feed and stow when appropriate
        superstructure.setDefaultCommand(either(
                        superstructure.runState(SuperstructureState.STOW),
                        superstructure.runState(SuperstructureState.FEED),
                        robotState::hasGamePiece)
                .withName("SuperstructureDefault"));
        climb.setDefaultCommand(climb.stop());

        teleop.onTrue(superstructure.hold(true));
        disabled.onTrue(superstructure.stop().ignoringDisable(true));

        streamDeck.configureButtons(
                config -> config.add(StreamDeckButton.L4_CORAL, () -> robotState.isSelected(CoralMode.L4_CORAL))
                        .add(StreamDeckButton.L3_CORAL, () -> robotState.isSelected(CoralMode.L3_CORAL))
                        .add(StreamDeckButton.L2_CORAL, () -> robotState.isSelected(CoralMode.L2_CORAL))
                        .add(StreamDeckButton.L1_CORAL, () -> robotState.isSelected(CoralMode.L1_CORAL))
                        .add(StreamDeckButton.ALGAE_INTAKE, () -> algaeIntake.isScheduled())
                        .add(StreamDeckButton.ALGAE_EJECT, () -> algaeEject.isScheduled())
                        .add(StreamDeckButton.FEED, () -> superstructure.getState() == SuperstructureState.FEED)
                        .add(StreamDeckButton.STOW, () -> superstructure.getState() == SuperstructureState.STOW)
                        .add(StreamDeckButton.BUMP_FORWARDS, () -> bumpForwards.isScheduled())
                        .add(StreamDeckButton.BUMP_REVERSE, () -> bumpReverse.isScheduled())
                        .add(StreamDeckButton.DEPLOY, () -> climbDeploy.isScheduled())
                        .add(StreamDeckButton.RETRACT, () -> climbRetract.isScheduled())
                        .add(StreamDeckButton.ZERO, () -> climbZero.isScheduled())
                        .add(StreamDeckButton.CORAL_INTAKE, () -> coralIntake.isScheduled())
                        .add(StreamDeckButton.CORAL_REJECT, () -> coralReject.isScheduled()));

        streamDeck.button(StreamDeckButton.L4_CORAL).onTrue(l4CoralScore);
        streamDeck.button(StreamDeckButton.L3_CORAL).onTrue(l3CoralScore);
        streamDeck.button(StreamDeckButton.L2_CORAL).onTrue(l2CoralScore);
        streamDeck.button(StreamDeckButton.L1_CORAL).onTrue(l1CoralScore);
        streamDeck.button(StreamDeckButton.ALGAE_INTAKE).onTrue(algaeIntake);
        streamDeck.button(StreamDeckButton.ALGAE_EJECT).onTrue(algaeEject);
        streamDeck.button(StreamDeckButton.FEED).onTrue(feed);
        streamDeck.button(StreamDeckButton.STOW).onTrue(stow);
        streamDeck.button(StreamDeckButton.BUMP_FORWARDS).onTrue(bumpForwards);
        streamDeck.button(StreamDeckButton.BUMP_REVERSE).onTrue(bumpReverse);
        streamDeck.button(StreamDeckButton.DEPLOY).onTrue(climbDeploy);
        streamDeck
                .button(StreamDeckButton.RETRACT)
                .onTrue(climbRetract)
                .onTrue(superstructure.runState(SuperstructureState.CLIMB));
        streamDeck.button(StreamDeckButton.ZERO).onTrue(climbZero);
        streamDeck.button(StreamDeckButton.CORAL_INTAKE).onTrue(coralIntake);
        streamDeck.button(StreamDeckButton.CORAL_REJECT).onTrue(coralReject);

        /* Dashboard Buttons */
        SmartDashboard.putData(superstructure.zeroElevator().withName("ElevatorZero"));
        SmartDashboard.putData(climbZero);

        /* Miscellaneous */
        teleop.onTrue(drive.resetRotation(robotState::getHeading));
        teleop.and(() -> robotState.getMatchTimer() > 20
                        && robotState.getMatchTimer() < 30
                        && robotState.isCountingDown()
                        && !climb.hasBeenZeroed())
                .onTrue(climbZero);

        var coastDebouncer = new Debouncer(10.0, DebounceType.kRising);
        new Trigger(() -> !DriverStation.isFMSAttached()
                        && coastDebouncer.calculate(DriverStation.isDisabled())
                        && coastButtonInput.get())
                .whileTrue(startEnd(
                                () -> {
                                    drive.setNeutralMode(NeutralModeValue.Coast);
                                    superstructure.setNeutralMode(NeutralModeValue.Coast);
                                    climb.setNeutralMode(NeutralModeValue.Coast);
                                },
                                () -> {
                                    drive.setNeutralMode(NeutralModeValue.Brake);
                                    superstructure.setNeutralMode(NeutralModeValue.Brake);
                                    climb.setNeutralMode(NeutralModeValue.Brake);
                                })
                        .ignoringDisable(true)
                        .withName("RobotCoastMode"));
    }

    private void configureAutoRoutines() {
        var superstructureScheduler =
                new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        var autoRoutines = new AutoRoutines(robotState, drive, superstructureScheduler);

        autoChooser.addRoutine("Right 4", autoRoutines::rightFour);
        autoChooser.addRoutine("Left 4", autoRoutines::leftFour);
        autoChooser.addRoutine("Ground Right 4", autoRoutines::groundRightFour);
        autoChooser.addRoutine("Ground Right 2", autoRoutines::groundRightTwo);
        autoChooser.addRoutine("Back Algae", autoRoutines::backAlgae);

        SmartDashboard.putData("Auto Chooser", autoChooser);

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

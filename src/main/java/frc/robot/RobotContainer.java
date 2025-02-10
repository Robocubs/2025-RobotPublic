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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.rollers.RollersIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOSim;
import frc.robot.util.TriggeredAlert;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
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
                            new Superstructure(new ElevatorIOSim(), new ArmIOSim(), new RollersIO() {}, robotState);
                    break;
            }
        }

        if (drive == null) {
            drive = new Drive(new DriveIO() {}, robotState);
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
    }

    private void configureAutoRoutines() {
        var autoRoutines = new AutoRoutines(robotState, drive);

        autoChooser.addRoutine("Demo", autoRoutines::demoAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.select("Demo");
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.controllers.PathController;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class Drive extends SubsystemBase {
    // Swerve drive requests
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyRobotSpeeds robotRequest = new SwerveRequest.ApplyRobotSpeeds();

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final RobotState robotState;
    private final PathController choreoTrajectoryController = new PathController();

    private Pose2d odometryPose = Pose2d.kZero;
    private double odometryPoseLastReset = 0.0;

    public Drive(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        var measurements = Stream.of(inputs.inputs)
                .map(input -> new DriveMeasurement(
                        input.timestamp,
                        input.rawHeading,
                        input.modulePositions,
                        kinematics.toChassisSpeeds(input.moduleStates)))
                .toArray(DriveMeasurement[]::new);
        robotState.addDriveMeasurements(measurements);

        if (inputs.inputs.length > 0) {
            var lastInput = inputs.inputs[inputs.inputs.length - 1];
            odometryPose = lastInput.pose;
        }
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public double getOdometryPoseLastReset() {
        return odometryPoseLastReset;
    }

    public void setRequest(SwerveRequest request) {
        io.setControl(request);
    }

    public void followPath(SwerveSample sample) {
        io.setControl(choreoTrajectoryController.update(sample, robotState.getPose()));
    }

    public Command resetRotation() {
        return resetRotation(() -> RobotState.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg);
    }

    public Command resetRotation(Supplier<Rotation2d> rotation) {
        return Commands.runOnce(() -> {
                    var rotationValue = rotation.get();
                    io.resetRotation(rotationValue);
                    odometryPose = new Pose2d(odometryPose.getTranslation(), rotationValue);
                    odometryPoseLastReset = Logger.getTimestamp();
                })
                .withName("DriveResetRotation");
    }

    public Command withSpeeds(ChassisSpeeds chassisSpeeds) {
        return run(() -> io.setControl(robotRequest.withSpeeds(chassisSpeeds))).withName("DriveWithSpeeds");
    }

    public Command withJoysticks(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier rotation) {
        return new DriveWithJoysticks(this, robotState, throttle, strafe, rotation);
    }

    public Command toPose(Supplier<Pose2d> poseSupplier, boolean finishAtGoal) {
        return new DriveToPose(this, robotState, poseSupplier, finishAtGoal);
    }

    public Command brake() {
        return run(() -> io.setControl(brakeRequest)).withName("DriveBrake");
    }

    public Command pointModulesAt(Supplier<Translation2d> target) {
        return run(() -> {
                    var pose = robotState.getPose();
                    var rotationToTarget =
                            target.get().minus(pose.getTranslation()).getAngle();
                    io.setControl(pointRequest.withModuleDirection(rotationToTarget.minus(pose.getRotation())));
                })
                .withName("DrivePointModules");
    }

    public Command pointModules(Supplier<Rotation2d> rotation) {
        return run(() -> io.setControl(pointRequest.withModuleDirection(rotation.get())))
                .withName("DrivePointModules");
    }
}

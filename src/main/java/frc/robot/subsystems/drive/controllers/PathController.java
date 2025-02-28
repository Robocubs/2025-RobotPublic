package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class PathController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public PathController() {
        xController = new PIDController(
                DriveConstants.translationP, DriveConstants.translationI, DriveConstants.translationD);
        yController = new PIDController(
                DriveConstants.translationP, DriveConstants.translationI, DriveConstants.translationD);
        rotationController =
                new PIDController(DriveConstants.rotationP, DriveConstants.rotationI, DriveConstants.rotationD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveRequest update(SwerveSample sample, Pose2d pose, LinearVelocity maxSpeed) {
        var targetPose = sample.getPose();

        var vx = sample.vx + xController.calculate(pose.getTranslation().getX(), targetPose.getX());
        var vy = sample.vy + yController.calculate(pose.getTranslation().getY(), targetPose.getY());
        var omega = sample.omega
                + rotationController.calculate(
                        pose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

        var speed = Math.hypot(vx, vy);
        if (speed > maxSpeed.in(MetersPerSecond)) {
            vx = maxSpeed.in(MetersPerSecond) / speed * vx;
            vy = maxSpeed.in(MetersPerSecond) / speed * vy;
        }

        Logger.recordOutput("Choreo/TargetPose", targetPose);

        return applyRobotSpeeds
                .withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation()))
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY());
    }
}

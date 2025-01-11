package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

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

    public SwerveRequest update(SwerveSample sample, Pose2d pose) {
        var targetPose = sample.getPose();

        var vx = sample.vx + xController.calculate(pose.getTranslation().getX(), targetPose.getX());
        var vy = sample.vy + yController.calculate(pose.getTranslation().getY(), targetPose.getY());
        var omega = sample.omega
                + rotationController.calculate(
                        pose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

        Logger.recordOutput("Choreo/TargetPose", targetPose);

        return applyRobotSpeeds
                .withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation()))
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY());
    }
}

package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.vision.VisionSubsystem;
import org.bitbuckets.vision.VisionUtil;
import org.photonvision.targeting.PhotonTrackedTarget;

import javax.swing.*;
import java.util.Optional;


// if it aint broke dont fix it
public class MoveToAlignCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final HolonomicDriveController holoController;
    final OdometrySubsystem odometrySubsystem;

    Pose3d targetPose;

    final double xThreshold = 0.05;
    final double yThreshold = 0.05;
    final double angleThreshold = 5;


    public MoveToAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, HolonomicDriveController holoController, OdometrySubsystem odometrySubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.holoController = holoController;
        this.odometrySubsystem = odometrySubsystem;
    }

    @Override
    public void initialize() {
        setTarget();
    }

    private void setTarget() {
        Optional<PhotonTrackedTarget> optTarget = visionSubsystem.getBestVisionTarget(true);
        if (optTarget.isPresent()) {
            this.targetPose = VisionUtil.getDesiredTargetAlignPose(optTarget.get());
            RobotContainer.VISION.log_desired_transform_pose(this.targetPose.toPose2d());
        }
    }
    @Override
    public void execute() {
        if (this.targetPose != null) {
            setTarget();

            double desiredVelocity = 0;
            
            ChassisSpeeds speeds = this.holoController.calculate(
                    this.odometrySubsystem.getRobotCentroidPosition(),
                    this.targetPose.toPose2d(),
                    desiredVelocity,
                    this.targetPose.toPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))
            );

        driveSubsystem.driveUsingChassisSpeed(speeds, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

    @Override
    public boolean isFinished() {
        if (this.targetPose != null) {
            Pose2d currentPose = odometrySubsystem.getRobotCentroidPosition();

            boolean finished =  Math.abs(currentPose.getX() - this.targetPose.getX()) < xThreshold &&
                    Math.abs(currentPose.getY() - this.targetPose.getY()) < yThreshold &&
                    (currentPose.getRotation().getDegrees() - this.targetPose.getRotation().getAngle()) < angleThreshold;


            return finished;
        } else return true;
    };
}

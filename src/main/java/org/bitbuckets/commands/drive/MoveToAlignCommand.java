package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.vision.VisionSubsystem;
import org.bitbuckets.vision.VisionUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

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

        // change this with visiontarget implementation later
        Optional<PhotonTrackedTarget> optTarget = visionSubsystem.getBestVisionTarget();


        if (optTarget.isPresent()) {
            Transform3d tagTransform = VisionUtil.getDesiredTargetAlignTransform(optTarget.get());
            this.targetPose = odometrySubsystem.getRobotCentroidPositionVert().plus(tagTransform);
        } else {
            this.targetPose = odometrySubsystem.getRobotCentroidPositionVert();
        }

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // this must be updated every as frequently as possible
        Optional<PhotonTrackedTarget> optTarget = visionSubsystem.getBestVisionTarget(true);


        if (optTarget.isPresent()) {
            Transform3d tagTransform = VisionUtil.getDesiredTargetAlignTransform(optTarget.get());
            this.targetPose = odometrySubsystem.getRobotCentroidPositionVert().plus(tagTransform);
        }

        moveToAlign();
    }
    public ChassisSpeeds calculateTagSpeeds(Pose2d target, Rotation2d holonomicRotation, double desiredVelocity) {
        return holoController.calculate(
                odometrySubsystem.getRobotCentroidPosition(),
                target,
                desiredVelocity,
                holonomicRotation
        );
    }

    public void moveToAlign() {

        ChassisSpeeds speeds = calculateTagSpeeds(this.targetPose.toPose2d(),
                this.targetPose.toPose2d().getRotation().plus(Rotation2d.fromDegrees(180)),
                1);
        driveSubsystem.driveUsingChassisSpeed(speeds);

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = odometrySubsystem.getRobotCentroidPosition();

        return Math.abs(currentPose.getX() - targetPose.getX()) < xThreshold &&
                Math.abs(currentPose.getY() - targetPose.getY()) < yThreshold &&
                (currentPose.getRotation().getDegrees() - targetPose.getRotation().getAngle()) < angleThreshold;
    }
}

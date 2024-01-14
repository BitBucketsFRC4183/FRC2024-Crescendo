package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.vision.VisionSubsystem;

import java.util.Optional;

public class MoveToAlignCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final HolonomicDriveController holonomicDriveController;


    public MoveToAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem, HolonomicDriveController holonomicDriveController) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.holonomicDriveController = holonomicDriveController;
    }

    public ChassisSpeeds calculatePose2dSpeeds(Pose2d targetPose, Rotation2d rotTarget, double velocity) {
        return holonomicDriveController.calculate(
                odometrySubsystem.getCurrentPose2d(),
                targetPose,
                velocity,
                rotTarget
       );
    }

    public void moveToAlign() {
        Optional<Pose3d> targetPose3d = visionSubsystem.estimateBestVisionTarget_1();
        if (targetPose3d.isPresent()) {
            Pose2d targetPose2d = targetPose3d.get().toPose2d();
            ChassisSpeeds calcSpeeds = calculatePose2dSpeeds(
                                            targetPose2d,
                                            targetPose2d.getRotation().plus(Rotation2d.fromDegrees(180)),
                                            1);
            driveSubsystem.driveUsingChassisSpeed(calcSpeeds);
        };


    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        moveToAlign();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.vision.VisionSubsystem;

import java.util.Optional;

public class MoveToAlignCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final HolonomicDriveController holoController;
    final OdometrySubsystem odometrySubsystem;
    final OperatorInput operatorInput;


    public MoveToAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, HolonomicDriveController holoControlleer, OdometrySubsystem odometrySubsystem, OperatorInput operatorInput) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.holoController = holoControlleer;
        this.odometrySubsystem = odometrySubsystem;
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        moveToAlign();
    }
    public ChassisSpeeds calculateTagSpeeds(Pose2d target, Rotation2d holonomicRotation, double desiredVelocity) {


        ChassisSpeeds speed = holoController.calculate(
                odometrySubsystem.getCurrentPosition(),
                target,
                desiredVelocity,
                holonomicRotation
        );

        double X_error = holoController.getXController().getPositionError();
        double Y_error = holoController.getYController().getPositionError();
        double theta_error = holoController.getThetaController().getPositionError();

        if ((X_error < 0.1 && X_error > -0.1) && (Y_error < 0.1 && Y_error > -0.1) && (theta_error < 5 && theta_error > -5)) {
            return new ChassisSpeeds(0, 0, 0);
        }


        return speed;
    }

    public void moveToAlign() {
        var tagPose_1 = visionSubsystem.estimateBestVisionTarget_1();
        if (tagPose_1.isPresent()) {
            ChassisSpeeds speeds = calculateTagSpeeds(tagPose_1.get().toPose2d(), tagPose_1.get().toPose2d().getRotation().plus(Rotation2d.fromDegrees(180)), 1);

            driveSubsystem.driveUsingChassisSpeed(speeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}

package org.bitbuckets.commands.drive;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

public class FollowTrajectoryCommand extends Command {
    double curTime_seconds = 0;
    final double TIME_MARGIN = 3; //margin of max time above estimated trajectory time
    final double TIMEOUT; //max time to execute trajectory before exiting


    final ChoreoTrajectory trajectory;
    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;

    final HolonomicDriveController holoController;

    public FollowTrajectoryCommand(ChoreoTrajectory trajectory, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, HolonomicDriveController holoController) {
        this.trajectory = trajectory;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.holoController = holoController;
        this.TIMEOUT = trajectory.getTotalTime() + TIME_MARGIN;

    }

    @Override
    public void execute() {
        curTime_seconds += 0.02;
        ChoreoTrajectoryState sample = trajectory.sample(curTime_seconds);
        ChassisSpeeds speeds = holoController.calculate(
                odometrySubsystem.getCurrentPosition(),
                sample.getPose(),
                Math.sqrt(Math.pow(sample.velocityX, 2) + Math.pow(sample.velocityY, 2)),
                sample.getPose().getRotation()
        );


        double X_error = holoController.getXController().getPositionError();
        double Y_error = holoController.getYController().getPositionError();
        double theta_error = holoController.getThetaController().getPositionError();

        if ((X_error < 0.1 && X_error > -0.1) && (Y_error < 0.1 && Y_error > -0.1) && (theta_error < 5 && theta_error > -5)) {
            speeds = new ChassisSpeeds(0, 0, 0);
        }
        ChassisSpeeds speedWithFeedback = new ChassisSpeeds(sample.x + speeds.vxMetersPerSecond, sample.y + speeds.vyMetersPerSecond , sample.angularVelocity + speeds.omegaRadiansPerSecond);

        driveSubsystem.driveUsingChassisSpeed(speedWithFeedback);
    }

    @Override
    public boolean isFinished() {
        //exit trajectory if it takes too long
        if (curTime_seconds > TIMEOUT){
            return true;
        }

        Pose2d desiredPoseAtTheEndOfTheTrajectory = trajectory.getFinalPose();
        Pose2d currentPose = odometrySubsystem.getCurrentPosition();
        boolean xCorrect = Math.abs(currentPose.getX() - desiredPoseAtTheEndOfTheTrajectory.getX()) > 0.2;
        boolean yCorrect = Math.abs(currentPose.getY() - desiredPoseAtTheEndOfTheTrajectory.getY()) > 0.2;
        boolean rotationCorrect = Math.abs(currentPose.getRotation().getRotations() - desiredPoseAtTheEndOfTheTrajectory.getRotation().getRotations()) > 0.014;
        return xCorrect && yCorrect && rotationCorrect;

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}

package org.bitbuckets.commands.drive;

import com.fasterxml.jackson.core.sym.ByteQuadsCanonicalizer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

import java.util.List;

public class FollowTrajectoryCommand extends Command {
    double curTime_seconds = 0;






    final Trajectory trajectory;
    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;

    final HolonomicDriveController holonomicDriveController;

    public FollowTrajectoryCommand(Trajectory trajectory, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, HolonomicDriveController holonomicDriveController) {
        this.trajectory = trajectory;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.holonomicDriveController = holonomicDriveController;

    }

    @Override
    public void execute() {
        curTime_seconds += 0.02;

        ChassisSpeeds speeds = holonomicDriveController.calculate(
                odometrySubsystem.getCurrentPosition(),
                trajectory.sample(curTime_seconds),
                trajectory.sample(curTime_seconds).poseMeters.getRotation()
        );

        driveSubsystem.driveUsingChassisSpeed(speeds);
    }

    @Override
    public boolean isFinished() {

        List<Trajectory.State> states = trajectory.getStates();
        Pose2d desiredPoseAtTheEndOfTheTrajectory = states.get(states.size() - 1).poseMeters;
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

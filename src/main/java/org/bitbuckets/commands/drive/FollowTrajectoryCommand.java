package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;

public class FollowTrajectoryCommand extends Command {

    final Trajectory trajectory;
    final DriveSubsystem driveSubsystem;

    final HolonomicDriveController holonomicDriveController;

    public FollowTrajectoryCommand(Trajectory trajectory, DriveSubsystem driveSubsystem, HolonomicDriveController holonomicDriveController) {
        this.trajectory = trajectory;
        this.driveSubsystem = driveSubsystem;
        this.holonomicDriveController = holonomicDriveController;
    }

    @Override
    public void execute() {
        /*ChassisSpeeds speeds;
        speeds = new

        driveSubsystem.driveUsingChassisSpeed();*/
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

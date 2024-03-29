package org.bitbuckets.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.Odometry;
import org.bitbuckets.vision.VisionSubsystem;

public class ToggleVisionOdometryCommand extends Command {

    final Odometry odometrySubsystem;

    public ToggleVisionOdometryCommand(Odometry odometrySubsystem) {
        this.odometrySubsystem = odometrySubsystem;
    }


    @Override
    public void execute() {
        odometrySubsystem.visionOdometry = !odometrySubsystem.visionOdometry;
        RobotContainer.VISION.log_vision_odometry_disabled(odometrySubsystem.visionOdometry);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

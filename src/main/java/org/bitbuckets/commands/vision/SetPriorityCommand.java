package org.bitbuckets.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.vision.VisionSubsystem;

public class SetPriorityCommand extends Command {

    final VisionSubsystem.VisionPriority priority;
    final VisionSubsystem visionSubsystem;

    public SetPriorityCommand(VisionSubsystem visionSubsystem, VisionSubsystem.VisionPriority priority) {
        this.priority = priority;
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }


    @Override
    public void execute() {
        visionSubsystem.priority = priority;
        RobotContainer.VISION.log_current_priority(priority.name());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

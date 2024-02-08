package org.bitbuckets.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.climber.ClimberSubsystem;
import org.bitbuckets.vision.VisionSubsystem;

public class SetPriority extends Command {

    final VisionSubsystem.VisionPriority priority;
    final VisionSubsystem visionSubsystem;

    public SetPriority(VisionSubsystem visionSubsystem, VisionSubsystem.VisionPriority priority) {
        this.priority = priority;
        this.visionSubsystem = visionSubsystem;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        visionSubsystem.priority = priority;
        RobotContainer.VISION.log_current_priority(priority.name());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return true;
    }
}

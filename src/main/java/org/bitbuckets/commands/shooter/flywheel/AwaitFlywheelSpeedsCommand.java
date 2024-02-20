package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class AwaitFlywheelSpeedsCommand extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final double thresholdSpeeds;

    public AwaitFlywheelSpeedsCommand(FlywheelSubsystem flywheelSubsystem, double thresholdSpeeds) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.thresholdSpeeds = thresholdSpeeds;
    }

    @Override public boolean isFinished() {
        return flywheelSubsystem.hasReachedSpeeds(thresholdSpeeds, thresholdSpeeds);
    }
}

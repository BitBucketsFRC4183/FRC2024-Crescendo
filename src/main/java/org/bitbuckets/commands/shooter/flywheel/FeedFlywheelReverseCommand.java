package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.shooter.PivotSubsystem;

public class FeedFlywheelReverseCommand extends Command {

    final FlywheelSubsystem flywheelSubsystem;

    public FeedFlywheelReverseCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
    }

    @Override public void execute() {
        flywheelSubsystem.setFlywheelVoltage(-6);
    }

    @Override public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelVoltage(0);
    }
}

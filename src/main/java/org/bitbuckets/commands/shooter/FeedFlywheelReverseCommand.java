package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class FeedFlywheelReverseCommand extends Command {

    final ShooterSubsystem shooterSubsystem;

    public FeedFlywheelReverseCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override public void execute() {
        shooterSubsystem.setAllMotorsToVoltage(-6);
    }

    @Override public void end(boolean interrupted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }
}

package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToSourceCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public PivotToSourceCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    // this angle needs to be tuned for amp (40 is only a placeholder)
    @Override
    public void execute() {
        shooterSubsystem.moveToRotation(0.125);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasReachedAngle(0.125);
    }
}

package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class IntakeCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;

    public IntakeCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.intake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.moveToRotation(0);
    }
}


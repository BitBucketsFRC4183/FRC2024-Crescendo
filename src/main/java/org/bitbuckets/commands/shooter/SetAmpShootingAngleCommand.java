package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SetAmpShootingAngleCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public SetAmpShootingAngleCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    // this angle needs to be tuned for amp (40 is only a placeholder)
    @Override
    public void execute() {
        shooterSubsystem.moveToAngle(45);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.moveToAngle(0);
    }
}

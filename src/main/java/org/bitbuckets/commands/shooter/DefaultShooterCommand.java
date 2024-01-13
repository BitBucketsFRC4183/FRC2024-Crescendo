package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class DefaultShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRotationalSpeeds(0, 0);
        shooterSubsystem.moveToAngle(45);
    }

    @Override
    public void end(boolean interrupted) {

    }
}

package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class ShootNoteCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;


    public ShootNoteCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRotationalSpeeds(1200, 1200);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasReachedSpeeds(1200, 1200);
    }

    @Override
    public void end(boolean isinteruppted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }

}

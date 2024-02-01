package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class AchieveFlatShotSpeedCommand extends Command {

    final ShooterSubsystem shooterSubsystem;


    public AchieveFlatShotSpeedCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRotationalSpeeds(5000, 5000);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasReachedSpeeds(5000, 5000);
    }

    @Override
    public void end(boolean isinteruppted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }

}

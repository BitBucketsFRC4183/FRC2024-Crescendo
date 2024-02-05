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
        shooterSubsystem.setMotorRotationalSpeeds(100,100);
        //shooterSubsystem.setMotorRotationalSpeeds(1,1);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasReachedSpeeds(100,100);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }

}

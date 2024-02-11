package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class AchieveFlatShotSpeedCommand extends Command {

    final ShooterSubsystem shooterSubsystem;
    final NoteManagementSubsystem noteManagementSubsystem;


    public AchieveFlatShotSpeedCommand(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRotationalSpeeds(500, 500);
        //shooterSubsystem.setMotorRotationalSpeeds(1,1);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasReachedSpeeds(500,500);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }

}

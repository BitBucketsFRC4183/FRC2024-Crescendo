package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.noteManagement.PushNoteFromNMSToShooterCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        super(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                new PushNoteFromNMSToShooterCommand(noteManagementSubsystem)
        );
    }
}

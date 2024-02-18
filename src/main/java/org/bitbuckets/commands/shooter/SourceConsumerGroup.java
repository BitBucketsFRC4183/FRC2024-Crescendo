package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SourceConsumerGroup extends ParallelDeadlineGroup {

    public SourceConsumerGroup(NoteManagementSubsystem noteManagementSubsystem, ShooterSubsystem shooterSubsystem) {
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new FeedFlywheelReverseCommand(shooterSubsystem)
        );
    }
}

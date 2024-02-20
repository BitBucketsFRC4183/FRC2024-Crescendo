package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.commands.shooter.flywheel.FeedFlywheelReverseCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.shooter.PivotSubsystem;

public class SourceConsumerGroup extends ParallelDeadlineGroup {

    public SourceConsumerGroup(NoteManagementSubsystem noteManagementSubsystem, FlywheelSubsystem flywheelSubsystem) {
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new FeedFlywheelReverseCommand(flywheelSubsystem)
        );
    }
}

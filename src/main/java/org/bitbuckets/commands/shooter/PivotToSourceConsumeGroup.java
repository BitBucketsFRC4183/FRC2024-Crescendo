package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToSourceConsumeGroup extends ParallelDeadlineGroup {

    public PivotToSourceConsumeGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new PivotToSourceCommand(shooterSubsystem),
                new FeedFlywheelReverseCommand(shooterSubsystem)
        );
    }

}

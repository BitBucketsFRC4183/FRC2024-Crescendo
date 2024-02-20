package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.commands.shooter.flywheel.FeedFlywheelReverseCommand;
import org.bitbuckets.commands.shooter.pivot.PivotToSourceCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.shooter.PivotSubsystem;

public class PivotToSourceConsumeGroup extends ParallelDeadlineGroup {

    public PivotToSourceConsumeGroup(FlywheelSubsystem flywheelSubsystem, PivotSubsystem pivotSubsystem, NoteManagementSubsystem noteManagementSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new PivotToSourceCommand(pivotSubsystem),
                new FeedFlywheelReverseCommand(flywheelSubsystem)
        );
    }

}

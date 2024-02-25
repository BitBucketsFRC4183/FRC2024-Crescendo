package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelVoltageIndefinite;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class SourceConsumerGroup extends ParallelDeadlineGroup {

    public SourceConsumerGroup(NoteManagementSubsystem noteManagementSubsystem, FlywheelSubsystem flywheelSubsystem) {
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new SpinFlywheelVoltageIndefinite(flywheelSubsystem, -7)
        );
    }
}

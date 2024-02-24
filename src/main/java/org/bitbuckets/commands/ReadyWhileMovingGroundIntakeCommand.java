package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.bitbuckets.commands.groundIntake.FeedGroundIntakeGroup;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class ReadyWhileMovingGroundIntakeCommand extends ParallelDeadlineGroup {

    public ReadyWhileMovingGroundIntakeCommand(Command trajectoryCommand, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        super(
                trajectoryCommand,
                new FeedGroundIntakeGroup(noteManagementSubsystem, groundIntakeSubsystem)
        );
    }
}

package org.bitbuckets.commands.groundIntake.spam;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class PulseGroundIntakeFor extends ParallelRaceGroup {

    public PulseGroundIntakeFor(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem, double dead) {
        super(
                Commands.waitSeconds(dead),
                new AwaitNoteInManagerCommand(noteManagementSubsystem),
                new PulseGroundIntake(groundIntakeSubsystem, noteManagementSubsystem)
        );
    }

}

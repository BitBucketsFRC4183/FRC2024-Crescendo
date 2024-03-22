package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class FeedGroundIntakeAutoGroup extends ParallelRaceGroup { //this finishes when awaitNoteInManager finishes


    public FeedGroundIntakeAutoGroup(NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double deadline) { //Race the two following commands
        super(
                Commands.waitSeconds(deadline),
                new AwaitNoteInManagerCommand(noteManagementSubsystem), //race these two
                new BasicGroundIntakeCommand(groundIntakeSubsystem, noteManagementSubsystem, 9, 2)
        );
    }

}
package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class FeedGroundIntakeGroup extends ParallelRaceGroup { //this finishes when awaitNoteInManager finishes


    public FeedGroundIntakeGroup(NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) { //Race the two following commands
        super(

                new SequentialCommandGroup(
                        new AwaitNoteInManagerCommand(noteManagementSubsystem), //race these two
                        new WaitCommand(0.3)
                ),
                new BasicGroundIntakeCommand(groundIntakeSubsystem, noteManagementSubsystem, 9, 2)
        );
    }

}
package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class LessAggressiveFeedGroundIntakeGroup extends ParallelRaceGroup { //this finishes when awaitNoteInManager finishes


    public LessAggressiveFeedGroundIntakeGroup(NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) { //Race the two following commands
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem), //race these two
                Commands.runEnd(
                        () -> { //run these during the command
                            groundIntakeSubsystem.setToVoltage(6);
                            noteManagementSubsystem.setAllToVoltage(2);
                        },
                        () -> {
                            groundIntakeSubsystem.setToVoltage(0);
                            noteManagementSubsystem.setAllToVoltage(0);
                        }//run this at the end
                )

        );
    }

}
package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class FinishGroundIntakeCommand extends ParallelRaceGroup { //this finishes when awaitNoteInManager finishes


    public FinishGroundIntakeCommand(NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) { //Race the two following commands
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem), //race these two
                Commands.runEnd(
                        () -> {noteManagementSubsystem.setAllToVoltage(12);
                            groundIntakeSubsystem.setToVoltage(7);
                            },

                        () -> groundIntakeSubsystem.setToVoltage(0) //run this at the end
                )

        );
    }

}


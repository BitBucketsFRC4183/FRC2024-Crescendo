package org.bitbuckets.commands.groundIntake.spam;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.commands.groundIntake.BasicGroundIntakeCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class PulseGroundIntake extends RepeatCommand {
    public PulseGroundIntake(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        super(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                Commands.waitSeconds(0.12),
                                new BasicGroundIntakeCommand(groundIntakeSubsystem, noteManagementSubsystem, 9, 2)
                        ),
                        Commands.waitSeconds(0.12)
                )
        );
    }
}

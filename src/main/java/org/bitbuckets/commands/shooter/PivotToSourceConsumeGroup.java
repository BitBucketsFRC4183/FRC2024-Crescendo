package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToSourceConsumeGroup extends SequentialCommandGroup {

    public PivotToSourceConsumeGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new ParallelCommandGroup(
                        new AwaitNoteInManagerCommand(noteManagementSubsystem),
                        new PivotToSourceCommand(shooterSubsystem),
                        new SpinFlywheelCommand(shooterSubsystem, true, flywheelSpeed_metersPerSecondOfFlywheel)
                )
        );
    }
}

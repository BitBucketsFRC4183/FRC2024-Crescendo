package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteNotInManagerCommand;
import org.bitbuckets.commands.noteManagement.FeedNMSCommand;
import org.bitbuckets.commands.shooter.flywheel.AwaitFlywheelSpeedsCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class FireWhenReadyGroup extends ParallelDeadlineGroup {

    //wait for the fly wheel to get up to speed then shoot. Cancel once the note leaves
    public FireWhenReadyGroup(FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new AwaitNoteNotInManagerCommand(noteManagementSubsystem),
                new SequentialCommandGroup(
                        new AwaitFlywheelSpeedsCommand(flywheelSubsystem, flywheelSpeed_metersPerSecondOfFlywheel),
                        new FeedNMSCommand(noteManagementSubsystem, groundIntakeSubsystem)
                )
        );
    }
}

package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.bitbuckets.commands.noteManagement.AwaitNoteNotInManagerCommand;
import org.bitbuckets.commands.noteManagement.FeedNMSCommand;
import org.bitbuckets.commands.shooter.flywheel.AwaitFlywheelSpeedsCommand;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelIndefinite;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;

/**
 *
 */
public class AmpMakeReadyGroup extends ParallelDeadlineGroup {

    public AmpMakeReadyGroup(FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new SequentialCommandGroup(
                        new AwaitNoteNotInManagerCommand(noteManagementSubsystem),
                        new WaitCommand(1)
                ),
                new SpinFlywheelIndefinite(flywheelSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel),
                new SequentialCommandGroup(
                        new AwaitFlywheelSpeedsCommand(flywheelSubsystem, flywheelSpeed_metersPerSecondOfFlywheel),
                        new WaitCommand(0.15), //TODO 0.15 is really important
                        new FeedNMSCommand(noteManagementSubsystem, groundIntakeSubsystem)

                )
        );
    }


}

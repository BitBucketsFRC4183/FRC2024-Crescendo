package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteNotInManagerCommand;
import org.bitbuckets.commands.noteManagement.FeedNMSCommand;
import org.bitbuckets.commands.shooter.flywheel.AwaitFlywheelSpeedsCommand;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelIndefinite;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
//import org.bitbuckets.shooter.PivotSubsystem;

/**
 * Call me FULL AUTO the way i got FIRE GROUPS
 */
/*public class PivotToPositionFireGroup extends ParallelDeadlineGroup {
    public PivotToPositionFireGroup(FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double position, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new AwaitNoteNotInManagerCommand(noteManagementSubsystem), //until we are done launching the note, do the following:
                new SpinFlywheelIndefinite(flywheelSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel), //spin the flywheel
                //new PivotToPositionIndefinite(pivotSubsystem, position), //move the pivot to the desired position
                new SequentialCommandGroup(
                        new ParallelCommandGroup( //wait for the pivot to reach the desired position and the flywheel to reach the desired speed
                                //new AwaitPivotPositionCommand(pivotSubsystem, position),
                                new AwaitFlywheelSpeedsCommand(flywheelSubsystem, flywheelSpeed_metersPerSecondOfFlywheel)
                        ),
                        new FeedNMSCommand(noteManagementSubsystem, groundIntakeSubsystem) //feed the note into the flywheel
                )
        );
    }
}*/

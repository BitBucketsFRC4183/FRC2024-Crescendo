package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelCommand;
import org.bitbuckets.commands.shooter.pivot.PivotToSpeakerCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.shooter.PivotSubsystem;

public class PivotToSpeakerFireGroup extends SequentialCommandGroup {
    public PivotToSpeakerFireGroup(FlywheelSubsystem flywheelSubsystem, PivotSubsystem pivotSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new ParallelCommandGroup(
                        new PivotToSpeakerCommand(pivotSubsystem),
                        new SpinFlywheelCommand(flywheelSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel)
                ),
                new FeedFlywheelAndFireGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, flywheelSpeed_metersPerSecondOfFlywheel)
        );
    }
}

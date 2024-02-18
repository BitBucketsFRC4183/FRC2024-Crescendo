package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToSpeakerFireGroup extends SequentialCommandGroup {
    public PivotToSpeakerFireGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new ParallelCommandGroup(
                        new PivotToSpeakerCommand(shooterSubsystem),
                        new SpinFlywheelCommand(shooterSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel)
                ),
                new FeedFlywheelAndFireGroup(shooterSubsystem, noteManagementSubsystem, groundIntakeSubsystem, flywheelSpeed_metersPerSecondOfFlywheel)
        );
    }
}

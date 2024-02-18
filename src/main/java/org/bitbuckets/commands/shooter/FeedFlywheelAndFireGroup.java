package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.groundIntake.FeedGroundIntakeGroup;
import org.bitbuckets.commands.noteManagement.FeedNMSCommand;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class FeedFlywheelAndFireGroup extends SequentialCommandGroup {

    public FeedFlywheelAndFireGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new FeedGroundIntakeGroup(noteManagementSubsystem, groundIntakeSubsystem),
                new SpinFlywheelCommand(shooterSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel),
                new FeedNMSCommand(noteManagementSubsystem, groundIntakeSubsystem)
        );
    }
}

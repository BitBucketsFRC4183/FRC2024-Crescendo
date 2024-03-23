package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.commands.groundIntake.FeedGroundIntakeAutoGroup;
import org.bitbuckets.commands.shooter.FireMakeReadyGroup;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class ReadyWhileMovingShootCommand extends SequentialCommandGroup {

    public ReadyWhileMovingShootCommand(Command followTrajectoryCommand, FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double ramFireSpeed, double deadline) {
        super(
                followTrajectoryCommand,
                new FeedGroundIntakeAutoGroup(noteManagementSubsystem, groundIntakeSubsystem, deadline),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed)
        );
    }


}

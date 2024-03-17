package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.commands.shooter.FireWhenReadyGroup;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelIndefinite;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class ReadyWhileMovingShootCommand extends ParallelDeadlineGroup {

    public ReadyWhileMovingShootCommand(Command followTrajectoryCommand, FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem, double ramFireSpeed, double deadline) {
        super(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                followTrajectoryCommand,
                                new FireWhenReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed)
                        ),
                        new WaitCommand(deadline)
                ),
                new SpinFlywheelIndefinite(flywheelSubsystem, false, ramFireSpeed)
        );
    }


}

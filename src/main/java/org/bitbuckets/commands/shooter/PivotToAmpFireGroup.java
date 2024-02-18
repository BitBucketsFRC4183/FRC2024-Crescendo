package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToAmpFireGroup extends SequentialCommandGroup {
    public PivotToAmpFireGroup(ShooterSubsystem shooterSubsystem, NoteManagementSubsystem noteManagementSubsystem, double flywheelSpeed_metersPerSecondOfFlywheel) {
        super(
                new ParallelCommandGroup(
                        new PivotToAmpCommand(shooterSubsystem),
                        new SpinFlywheelCommand(shooterSubsystem, false, flywheelSpeed_metersPerSecondOfFlywheel)
                ),
                new FeedFlywheelAndFireGroup(shooterSubsystem, noteManagementSubsystem, flywheelSpeed_metersPerSecondOfFlywheel)
        );
    }
}

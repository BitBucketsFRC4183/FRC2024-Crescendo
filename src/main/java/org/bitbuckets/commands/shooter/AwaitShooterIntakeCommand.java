package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.bitbuckets.commands.noteManagement.AwaitNoteInManagerCommand;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class AwaitShooterIntakeCommand extends ParallelRaceGroup { //this finishes when awaitNoteInManager finishes


    public AwaitShooterIntakeCommand(NoteManagementSubsystem noteManagementSubsystem, ShooterSubsystem shooterSubsystem) { //Race the two following commands
        super(
                new AwaitNoteInManagerCommand(noteManagementSubsystem), //race these two
                Commands.runEnd(
                        () -> { //run these during the command
                            shooterSubsystem.setMotorRotationalSpeeds(-4000, -4000);
                            shooterSubsystem.moveToRotation(0.125);
                        },
                        () -> shooterSubsystem.setAllMotorsToVoltage(0) //run this at the end
                )

        );
    }

}
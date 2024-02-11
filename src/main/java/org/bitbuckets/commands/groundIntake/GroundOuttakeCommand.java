package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class GroundOuttakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;
    final NoteManagementSubsystem noteManagementSubsystem;


    public GroundOuttakeCommand(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.noteManagementSubsystem = noteManagementSubsystem;
    }


    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(-7);
        noteManagementSubsystem.setAllToVoltage(-3); //TODO FLIP
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
    }

}

package org.bitbuckets.commands.noteManagement;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class FeedNMSCommand extends Command {

    final NoteManagementSubsystem noteManagementSubsystem;
    final GroundIntakeSubsystem groundIntakeSubsystem;

    public FeedNMSCommand(NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        this.noteManagementSubsystem = noteManagementSubsystem;
        this.groundIntakeSubsystem = groundIntakeSubsystem;
    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(7);
        noteManagementSubsystem.setAllToVoltage(2);
    }

    @Override
    public boolean isFinished() {
        return !noteManagementSubsystem.isNoteIn();
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
        noteManagementSubsystem.setAllToVoltage(0);
    }
}

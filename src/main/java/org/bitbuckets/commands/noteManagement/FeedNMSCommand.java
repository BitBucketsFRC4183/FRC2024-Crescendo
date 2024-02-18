package org.bitbuckets.commands.noteManagement;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class FeedNMSCommand extends Command {

    final NoteManagementSubsystem noteManagementSubsystem;

    public FeedNMSCommand(NoteManagementSubsystem noteManagementSubsystem) {
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void execute() {
        noteManagementSubsystem.setAllToVoltage(2);
    }

    @Override
    public boolean isFinished() {
        return !noteManagementSubsystem.isNoteIn();
    }

    @Override
    public void end(boolean interrupted) {
        noteManagementSubsystem.setAllToVoltage(0);
    }
}

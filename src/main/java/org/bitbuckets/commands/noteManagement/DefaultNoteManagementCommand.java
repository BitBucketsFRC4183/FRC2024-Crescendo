package org.bitbuckets.commands.noteManagement;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class DefaultNoteManagementCommand extends Command {

    private final NoteManagementSubsystem noteManagementSubsystem;

    public DefaultNoteManagementCommand(NoteManagementSubsystem noteManagementSubsystem) {
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //rotations per second are placeholder values
    }

    @Override
    public void end(boolean interrupted) {
        noteManagementSubsystem.setAllMotorsToVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

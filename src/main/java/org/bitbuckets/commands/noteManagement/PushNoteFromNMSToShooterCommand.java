package org.bitbuckets.commands.noteManagement;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class PushNoteFromNMSToShooterCommand extends Command {

    final NoteManagementSubsystem noteManagementSubsystem;

    public PushNoteFromNMSToShooterCommand(NoteManagementSubsystem noteManagementSubsystem) {
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void execute() {
        noteManagementSubsystem.setAllToVoltage(12);
    }



    @Override
    public boolean isFinished() {
        return !noteManagementSubsystem.isNoteIn() && noteManagementSubsystem.motorsAtVelocity(50);
    }

    @Override
    public void end(boolean interrupted) {
        noteManagementSubsystem.setAllToVoltage(0);
    }
}

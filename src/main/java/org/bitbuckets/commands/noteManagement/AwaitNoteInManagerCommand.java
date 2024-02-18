package org.bitbuckets.commands.noteManagement;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class AwaitNoteInManagerCommand extends Command {

    final NoteManagementSubsystem nms;

    public AwaitNoteInManagerCommand(NoteManagementSubsystem nms) {
        this.nms = nms;

        addRequirements(nms);
    }

    @Override
    public boolean isFinished() {
        return nms.isNoteIn();
    }
}

package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class BasicGroundIntakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;
    final NoteManagementSubsystem noteManagementSubsystem;

    public BasicGroundIntakeCommand(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(10);
        noteManagementSubsystem.setAllToVoltage(5);
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
        noteManagementSubsystem.setAllToVoltage(0);
    }

}

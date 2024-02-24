package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

public class BasicGroundIntakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;
    final NoteManagementSubsystem noteManagementSubsystem;
    final double groundIntakeVoltage;
    final double noteManagementVoltage;

    public BasicGroundIntakeCommand(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem, double groundIntakeVoltage, double noteManagementVoltage) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.noteManagementSubsystem = noteManagementSubsystem;
        this.groundIntakeVoltage = groundIntakeVoltage;
        this.noteManagementVoltage = noteManagementVoltage;
    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(groundIntakeVoltage); //7
        noteManagementSubsystem.setAllToVoltage(noteManagementVoltage); //2
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
        noteManagementSubsystem.setAllToVoltage(0);
    }

}

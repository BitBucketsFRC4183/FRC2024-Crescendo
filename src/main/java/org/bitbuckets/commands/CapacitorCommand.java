package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;

import java.nio.LongBuffer;

public class CapacitorCommand extends Command {
    final LongBuffer timeBuf = LongBuffer.allocate(100);
    final GroundIntakeSubsystem groundIntakeSubsystem;
    final NoteManagementSubsystem noteManagementSubsystem;

    public CapacitorCommand(GroundIntakeSubsystem groundIntakeSubsystem, NoteManagementSubsystem noteManagementSubsystem) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.noteManagementSubsystem = noteManagementSubsystem;
    }

    @Override
    public void execute() {

        for(int i = 0; i < 100; i++) {
            timeBuf.put(System.currentTimeMillis());
        }
        long timePassed_norm = (System.currentTimeMillis() - timeBuf.get(0))/100;
        groundIntakeSubsystem.setToVoltage(10*(1-(timePassed_norm)));
        noteManagementSubsystem.setAllToVoltage(2*(1-timePassed_norm));


    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
        noteManagementSubsystem.setAllToVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return !timeBuf.hasRemaining();
    }
}

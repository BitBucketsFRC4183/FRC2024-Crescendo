package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.led.LedSubsystem;

public class LedNoteInCommand extends Command {
    final LedSubsystem ledSubsystem;

    public LedNoteInCommand(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }

    @Override public void initialize() {
        ledSubsystem.setColor("red");
    }

    @Override public void end(boolean interrupted) {
        ledSubsystem.turnOff();
    }
}

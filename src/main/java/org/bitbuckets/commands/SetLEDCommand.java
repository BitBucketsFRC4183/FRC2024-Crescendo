package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.LedSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class SetLEDCommand extends Command {

    final Color toSetTo;
    final LedSubsystem ledSubsystem;


    public SetLEDCommand(Color toSetTo, LedSubsystem ledSubsystem) {
        this.toSetTo = toSetTo;

        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void execute() {

        System.out.println("AAAAAA");
        ledSubsystem.setColor(toSetTo);
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setColor(Color.kRed);
    }
}

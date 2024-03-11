package org.bitbuckets.commands.amp;

import org.bitbuckets.amp.Amp;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractAmpCommand extends Command {
    final Amp ampSubsystem;

    public RetractAmpCommand(Amp ampSubsystem) {
        this.ampSubsystem = ampSubsystem;
    }


    @Override
    public void execute(){
        ampSubsystem.retract();
    }

    @Override
    public boolean isFinished(){
        return (ampSubsystem.hasReachedInitial());
    }

    @Override
    public void end(boolean interrupted){
        ampSubsystem.doNothing();
    }
}


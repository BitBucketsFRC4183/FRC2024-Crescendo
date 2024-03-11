package org.bitbuckets.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.amp.Amp;

public class MakeAmpReadyCommand extends Command {
  final Amp ampSubsystem;

  public MakeAmpReadyCommand(Amp ampSubsystem) {
    this.ampSubsystem = ampSubsystem;
  }


  @Override
  public void execute(){
    ampSubsystem.moveToAmp();
  }

  @Override
  public boolean isFinished(){
    return (ampSubsystem.hasReachedAmp());
  }

  @Override
  public void end(boolean interrupted){
    ampSubsystem.setToZero(); 

  }


}


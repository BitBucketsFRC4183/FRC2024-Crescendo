package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;

public class VibratingCommand extends Command {

    final OperatorInput oi;

    public VibratingCommand(OperatorInput oi) {
        this.oi = oi;
    }

    @Override public void initialize() {
        oi.setOperatorVibrating(true);
    }

    @Override public void end(boolean interrupted) {
        oi.setOperatorVibrating(false);
    }
}

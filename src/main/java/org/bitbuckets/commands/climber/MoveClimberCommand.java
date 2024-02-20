package org.bitbuckets.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.climber.ClimberSubsystem;

public class MoveClimberCommand extends Command {

    final ClimberSubsystem climberSubsystem;
    final OperatorInput operatorInput;

    public MoveClimberCommand(ClimberSubsystem climberSubsystem, OperatorInput operatorInput) {
        this.climberSubsystem = climberSubsystem;
        this.operatorInput = operatorInput;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climberSubsystem.setToVoltage(operatorInput.getClimberInput() * 12); // TODO multiply by dampener
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setToVoltage(0);
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}

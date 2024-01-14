package org.bitbuckets.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.climber.ClimberSubsystem;

public class LiftClimberCommand extends Command {

    final ClimberSubsystem climberSubsystem;
    final OperatorInput operatorInput;

    public LiftClimberCommand(ClimberSubsystem climberSubsystem, OperatorInput operatorInput) {
        this.climberSubsystem = climberSubsystem;
        this.operatorInput = operatorInput;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climberSubsystem.setFFElevatorSpeeds(operatorInput.getClimberInput(), operatorInput.getClimberInput());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return false;
    }
}

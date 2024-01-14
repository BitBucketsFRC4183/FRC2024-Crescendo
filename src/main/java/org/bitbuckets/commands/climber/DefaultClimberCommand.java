package org.bitbuckets.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.climber.ClimberSubsystem;

public class DefaultClimberCommand extends Command {

    final ClimberSubsystem climberSubsystem;

    public DefaultClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

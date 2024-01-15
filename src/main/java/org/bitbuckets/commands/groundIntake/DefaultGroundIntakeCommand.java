package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;

public class DefaultGroundIntakeCommand extends Command {

    private final GroundIntakeSubsystem groundIntakeSubsystem;

    public DefaultGroundIntakeCommand(GroundIntakeSubsystem groundIntakeSubsystem) {
        this.groundIntakeSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setMotorsZero();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }



}

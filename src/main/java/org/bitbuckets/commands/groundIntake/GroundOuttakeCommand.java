package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;

public class GroundOuttakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;
    final OperatorInput operatorInput;


    public GroundOuttakeCommand(GroundIntakeSubsystem groundIntakeSubsystem, OperatorInput operatorInput) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.operatorInput = operatorInput;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(-12); //TODO tune voltage for intake
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

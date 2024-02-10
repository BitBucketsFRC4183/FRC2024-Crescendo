package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;

public class GroundOuttakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;


    public GroundOuttakeCommand(GroundIntakeSubsystem groundIntakeSubsystem) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
    }


    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(-7); //TODO tune voltage for intake
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
    }

}

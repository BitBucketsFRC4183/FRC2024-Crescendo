package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.shooter.PivotSubsystem;

public class ManualPivotCommand extends Command {

    final OperatorInput oi;
    final PivotSubsystem pivotSubsystem;


    public ManualPivotCommand(OperatorInput oi, PivotSubsystem pivotSubsystem) {
        this.oi = oi;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }


    @Override
    public void execute() {
        double angleVoltageConstant = 4d;
        pivotSubsystem.setPivotMotorToVoltage(oi.getOperatorLeftStickY() * angleVoltageConstant);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setPivotMotorToVoltage(0);
    }

}

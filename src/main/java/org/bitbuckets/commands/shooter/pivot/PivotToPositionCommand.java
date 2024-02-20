package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.PivotSubsystem;

import java.util.function.Supplier;

public class PivotToPositionCommand extends Command {

    final PivotSubsystem pivotSubsystem;
    final double angle_mechanismRotations;

    public PivotToPositionCommand(PivotSubsystem pivotSubsystem, double angleMechanismRotations) {
        this.pivotSubsystem = pivotSubsystem;
        angle_mechanismRotations = angleMechanismRotations;
    }


    // this angle needs to be tuned for amp (40 is only a placeholder)
    @Override
    public void execute() {
        pivotSubsystem.moveToRotation(angle_mechanismRotations);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.hasReachedAngle(angle_mechanismRotations);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setPivotMotorToZero();
    }
}

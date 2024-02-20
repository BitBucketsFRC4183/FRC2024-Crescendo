package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.PivotSubsystem;

public class PivotToPositionIndefinite extends Command {

    final PivotSubsystem pivotSubsystem;
    final double angle_mechanismRotations;

    public PivotToPositionIndefinite(PivotSubsystem pivotSubsystem, double angleMechanismRotations) {
        this.pivotSubsystem = pivotSubsystem;
        angle_mechanismRotations = angleMechanismRotations;
    }


    @Override
    public void execute() {
        pivotSubsystem.moveToRotation(angle_mechanismRotations);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setPivotMotorToZero();
    }
}

package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.PivotSubsystem;

public class AwaitPivotPositionCommand extends Command {

    final PivotSubsystem pivotSubsystem;
    final double pivotPosition_mechanismRotations;

    public AwaitPivotPositionCommand(PivotSubsystem pivotSubsystem, double pivotPositionMechanismRotations) {
        this.pivotSubsystem = pivotSubsystem;
        this.pivotPosition_mechanismRotations = pivotPositionMechanismRotations;
    }

    @Override public boolean isFinished() {
        return pivotSubsystem.hasReachedAngle(pivotPosition_mechanismRotations);
    }
}

package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.PivotSubsystem;

public class PivotToAmpCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    public PivotToAmpCommand(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
    }

    // this angle needs to be tuned for amp (40 is only a placeholder)
    @Override
    public void execute() {
        pivotSubsystem.moveToRotation(0.125);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.hasReachedAngle(0.125);
    }
}

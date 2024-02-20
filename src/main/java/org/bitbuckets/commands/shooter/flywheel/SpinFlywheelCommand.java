package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.shooter.PivotSubsystem;

public class SpinFlywheelCommand extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final boolean inverted;
    final double flatShotFlywheelSpeed_mechanismRotationsPerSecond;


    public SpinFlywheelCommand(FlywheelSubsystem flywheelSubsystem, boolean inverted, double flatShotSpeed_mechanismRotationsPerSecond) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.inverted = inverted;
        flatShotFlywheelSpeed_mechanismRotationsPerSecond = flatShotSpeed_mechanismRotationsPerSecond;
    }

    @Override
    public void execute() {
        double invertedCoef = inverted ? -1 : 1;
        //shooterSubsystem.setAllMotorsToVoltage(10);
        //TODO fix this

        flywheelSubsystem.setFlywheelSpeeds(flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef, flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef);
    }

    @Override
    public boolean isFinished() {

        double invertedCoef = inverted ? -1 : 1;
        return flywheelSubsystem.hasReachedSpeeds(flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef, flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelVoltage(0);
    }

}

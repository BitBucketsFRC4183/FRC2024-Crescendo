package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class SpinFlywheelIndefinite extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final boolean inverted;
    final double flatShotFlywheelSpeed_mechanismRotationsPerSecond;


    public SpinFlywheelIndefinite(FlywheelSubsystem flywheelSubsystem, boolean inverted, double flatShotSpeed_mechanismRotationsPerSecond) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.inverted = inverted;
        flatShotFlywheelSpeed_mechanismRotationsPerSecond = flatShotSpeed_mechanismRotationsPerSecond;
    }

    @Override public void initialize() {
        flywheelSubsystem.insertDesiredDisplaySpeeds(flatShotFlywheelSpeed_mechanismRotationsPerSecond);
    }

    @Override
    public void execute() {
        double invertedCoef = inverted ? -1 : 1;

        flywheelSubsystem.setFlywheelSpeeds(flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef, flatShotFlywheelSpeed_mechanismRotationsPerSecond * invertedCoef);
    }


    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelVoltage(0);
    }

}

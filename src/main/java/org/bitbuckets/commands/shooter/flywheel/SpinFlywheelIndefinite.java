package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;

import java.util.function.Supplier;

public class SpinFlywheelIndefinite extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final boolean inverted;
    final Supplier<Double> flatShotFlywheelSpeed_mechanismRotationsPerSecond;


    public SpinFlywheelIndefinite(FlywheelSubsystem flywheelSubsystem, boolean inverted, Supplier<Double> flatShotSpeed_mechanismRotationsPerSecond) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.inverted = inverted;
        flatShotFlywheelSpeed_mechanismRotationsPerSecond = flatShotSpeed_mechanismRotationsPerSecond;
    }

    @Override
    public void execute() {
        double invertedCoef = inverted ? -1 : 1;

        flywheelSubsystem.setFlywheelSpeeds(flatShotFlywheelSpeed_mechanismRotationsPerSecond.get() * invertedCoef, flatShotFlywheelSpeed_mechanismRotationsPerSecond.get()* invertedCoef);
    }


    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelVoltage(0);
    }

}

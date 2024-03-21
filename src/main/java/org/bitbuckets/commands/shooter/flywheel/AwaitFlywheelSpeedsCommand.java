package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;

import java.util.function.Supplier;

public class AwaitFlywheelSpeedsCommand extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final Supplier<Double> thresholdSpeeds;

    public AwaitFlywheelSpeedsCommand(FlywheelSubsystem flywheelSubsystem, Supplier<Double> thresholdSpeeds) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.thresholdSpeeds = thresholdSpeeds;
    }

    @Override public void initialize() {
        System.out.println("speedsP " + thresholdSpeeds);
    }

    @Override public boolean isFinished() {
        return flywheelSubsystem.hasReachedSpeeds(thresholdSpeeds.get(), thresholdSpeeds.get());
    }
}

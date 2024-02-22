package org.bitbuckets.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.FlywheelSubsystem;

public class SpinFlywheelVoltageIndefinite extends Command {

    final FlywheelSubsystem flywheelSubsystem;
    final double voltage;

    public SpinFlywheelVoltageIndefinite(FlywheelSubsystem flywheelSubsystem, double voltage) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.voltage = voltage;
    }

    @Override public void execute() {
        flywheelSubsystem.setFlywheelVoltage(voltage);
    }

    @Override public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelVoltage(0);
    }
}

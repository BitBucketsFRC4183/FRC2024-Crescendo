package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SpinFlywheelCommand extends Command {

    final ShooterSubsystem shooterSubsystem;
    final boolean inverted;
    final double flatShotFlywheelSpeed_metersPerSecondOfFlywheel;


    public SpinFlywheelCommand(ShooterSubsystem shooterSubsystem, boolean inverted, double flatShotFlywheelSpeedMetersPerSecondOfFlywheel) {
        this.shooterSubsystem = shooterSubsystem;
        this.inverted = inverted;
        flatShotFlywheelSpeed_metersPerSecondOfFlywheel = flatShotFlywheelSpeedMetersPerSecondOfFlywheel;
    }

    @Override
    public void execute() {
        double invertedCoef = inverted ? -1 : 1;
        shooterSubsystem.setMotorRotationalSpeeds(flatShotFlywheelSpeed_metersPerSecondOfFlywheel * invertedCoef, flatShotFlywheelSpeed_metersPerSecondOfFlywheel * invertedCoef);
    }

    @Override
    public boolean isFinished() {
        double invertedCoef = inverted ? -1 : 1;
        return shooterSubsystem.hasReachedSpeeds(flatShotFlywheelSpeed_metersPerSecondOfFlywheel * invertedCoef,flatShotFlywheelSpeed_metersPerSecondOfFlywheel * invertedCoef);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setAllMotorsToVoltage(0);
    }

}

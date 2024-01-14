package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SetShootingAngleManuallyCommand extends Command {

    final OperatorInput oi;
    final ShooterSubsystem shooterSubsystem;


    public SetShootingAngleManuallyCommand(OperatorInput oi, ShooterSubsystem shooterSubsystem) {
        this.oi = oi;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double angleVoltageConstant = 1d;
        if (shooterSubsystem.getPivotAnglePosition_normalizedMechanismRotations() > 0.001) {
            shooterSubsystem.setPivotMotorToVoltage(oi.getOperatorLeftStickY() * angleVoltageConstant);
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.shooter.ShooterSubsystem;

public class ManualPivotCommand extends Command {

    final OperatorInput oi;
    final ShooterSubsystem shooterSubsystem;


    public ManualPivotCommand(OperatorInput oi, ShooterSubsystem shooterSubsystem) {
        this.oi = oi;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }



    @Override
    public void execute() {
        double angleVoltageConstant = 4d;
        if (shooterSubsystem.getPivotAnglePosition_normalizedMechanismRotations() > 0.001) {
            shooterSubsystem.setPivotMotorToVoltage(oi.getOperatorLeftStickY() * angleVoltageConstant);
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setPivotMotorToVoltage(0);
    }

}

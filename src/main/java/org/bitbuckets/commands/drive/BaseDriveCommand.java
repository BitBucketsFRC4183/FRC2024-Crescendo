package org.bitbuckets.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.OperatorToSpeeds;
import org.bitbuckets.drive.DriveSubsystem;

public class BaseDriveCommand extends Command {

    final OperatorToSpeeds operatorToSpeeds;
    final OperatorInput operatorInput;
    final DriveSubsystem swerveSubsystem;

    public BaseDriveCommand(OperatorToSpeeds operatorToSpeeds, OperatorInput operatorInput, DriveSubsystem swerveSubsystem) {
        this.operatorToSpeeds = operatorToSpeeds;
        this.operatorInput = operatorInput;
        this.swerveSubsystem = swerveSubsystem;

        xBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1)).negate();
        yBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)).negate();
        rotBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1)).negate();

        addRequirements(swerveSubsystem);

    }

    final Trigger xBad;
    final Trigger yBad;
    final Trigger rotBad;



    @Override public void execute() {
        ChassisSpeeds limitedSpeeds = operatorToSpeeds.desiredLimitedSpeeds_fieldRelative(
                swerveSubsystem.odometry.getHeading_fieldRelative()
        );

        if (xBad.getAsBoolean()
                && yBad.getAsBoolean()
                && rotBad.getAsBoolean()
        ) return;

        swerveSubsystem.orderToUnfiltered(limitedSpeeds);
    }

}

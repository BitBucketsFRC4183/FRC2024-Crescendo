package org.bitbuckets.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.OperatorToSpeeds;
import org.bitbuckets.drive.SwerveSubsystem;

public class EddieFacingCommand extends Command {

    final OperatorToSpeeds operatorToSpeeds;
    final OperatorInput operatorInput;
    final SwerveSubsystem swerveSubsystem;

    public EddieFacingCommand(OperatorToSpeeds operatorToSpeeds, OperatorInput operatorInput, SwerveSubsystem swerveSubsystem) {
        this.operatorToSpeeds = operatorToSpeeds;
        this.operatorInput = operatorInput;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override public void execute() {

        ChassisSpeeds limitedSpeeds = operatorToSpeeds.desiredLimitedSpeeds_fieldRelative(
                swerveSubsystem.odometry.getHeading_fieldRelative()
        );

        //FLIP IF WRONG TEAM



        if (operatorInput.frontSpeakerHeadingHold.getAsBoolean()) {
            //handle flipping
            Rotation2d headingForFlippingOnly = Rotation2d.fromDegrees(0);
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (shouldFlip) {
                headingForFlippingOnly = headingForFlippingOnly.plus(Rotation2d.fromDegrees(180));
            }

            swerveSubsystem.orderTo(limitedSpeeds, headingForFlippingOnly);
            return;
        }

        if (operatorInput.leftSpeakerHeadingHold.getAsBoolean()) {
            //handle flipping
            Rotation2d headingForFlippingOnly = Rotation2d.fromRadians(-Math.PI / 3);
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (shouldFlip) {
                headingForFlippingOnly = headingForFlippingOnly.plus(Rotation2d.fromDegrees(180));
            }

            swerveSubsystem.orderTo(limitedSpeeds, headingForFlippingOnly);
            return;
        }

        if (operatorInput.rightSpeakerHeadingHold.getAsBoolean()) {
            //handle flipping
            Rotation2d headingForFlippingOnly = Rotation2d.fromRadians(Math.PI / 3);
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (shouldFlip) {
                headingForFlippingOnly = headingForFlippingOnly.plus(Rotation2d.fromDegrees(180));
            }

            swerveSubsystem.orderTo(limitedSpeeds, headingForFlippingOnly);
            return;
        }

        if (operatorInput.ampHeadingHold.getAsBoolean()) {
            //handle flipping
            Rotation2d headingForFlippingOnly = Rotation2d.fromDegrees(-90);
            swerveSubsystem.orderTo(limitedSpeeds, headingForFlippingOnly);
            return;
        }



        swerveSubsystem.orderTo(limitedSpeeds);
    }

    @Override public void end(boolean interrupted) {
        swerveSubsystem.orderToZero();
    }
}

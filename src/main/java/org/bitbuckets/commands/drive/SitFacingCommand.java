package org.bitbuckets.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.SwerveSubsystem;
import xyz.auriium.mattlib2.auto.pid.IPIDController;
import xyz.auriium.mattlib2.auto.pid.RotationalPIDBrain;

public class SitFacingCommand extends Command {

    final RotationalPIDBrain pidBrain;
    final SwerveSubsystem swerveSubsystem;
    final Rotation2d desiredHeading_allianceRelative;

    IPIDController controller;

    public SitFacingCommand(RotationalPIDBrain pidBrain, SwerveSubsystem swerveSubsystem, Rotation2d desiredHeadingAllianceRelative) {
        this.pidBrain = pidBrain;
        this.swerveSubsystem = swerveSubsystem;
        desiredHeading_allianceRelative = desiredHeadingAllianceRelative;
    }

    @Override public void initialize() {
        controller = pidBrain.spawn();
    }

    @Override public void execute() {
        Rotation2d headingForFlippingOnly = desiredHeading_allianceRelative;
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        if (shouldFlip) {
            headingForFlippingOnly = headingForFlippingOnly.plus(Rotation2d.fromDegrees(180));
        }

        double controlOut = controller.controlToReference_primeUnits(headingForFlippingOnly.getRadians(), swerveSubsystem.odometry.getHeading_fieldRelative().getRadians());

        swerveSubsystem.orderToUnfiltered(new ChassisSpeeds(0,0,controlOut));
    }
}

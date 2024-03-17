package org.bitbuckets.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.auto.pid.IPIDController;
import xyz.auriium.mattlib2.auto.pid.RotationalPIDBrain;

public class SitFacingCommand extends Command {

    final RotationalPIDBrain pidBrain;
    final DriveSubsystem swerveSubsystem;
    final Rotation2d desiredHeading_allianceOrField;
    final boolean isAllianceRelative;

    IPIDController controller;
    Rotation2d heading;

    public SitFacingCommand(RotationalPIDBrain pidBrain, DriveSubsystem swerveSubsystem, Rotation2d desiredHeadingAllianceRelative, boolean isAllianceRelative) {
        this.pidBrain = pidBrain;
        this.swerveSubsystem = swerveSubsystem;
        desiredHeading_allianceOrField = desiredHeadingAllianceRelative;
        this.isAllianceRelative = isAllianceRelative;

        addRequirements(swerveSubsystem);
    }

    @Override public void initialize() {
        controller = pidBrain.spawn();

        heading = desiredHeading_allianceOrField;
        if (isAllianceRelative) {
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (shouldFlip) {
                heading = heading.plus(Rotation2d.fromDegrees(180));
            }
        }
    }

    @Override public void execute() {
        double controlOut = controller.controlToReference_primeUnits(heading.getRadians(), swerveSubsystem.odometry.getHeading_fieldRelative().getRadians());

        swerveSubsystem.orderToUnfiltered(new ChassisSpeeds(0,0,controlOut));
    }

    @Override public void end(boolean interrupted) {
        controller = null;
    }
}

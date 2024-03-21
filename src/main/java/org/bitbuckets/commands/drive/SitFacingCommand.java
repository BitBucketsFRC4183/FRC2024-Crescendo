package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.auto.pid.IPIDController;
import xyz.auriium.mattlib2.auto.pid.RotationalPIDBrain;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

public class SitFacingCommand extends Command {

    final PIDComponent pidBrain;
    final DriveSubsystem swerveSubsystem;
    final Rotation2d desiredHeading_allianceOrField;
    final boolean isAllianceRelative;

    final PIDController rotationalController = new PIDController(0,0,0);


    Rotation2d heading;

    public SitFacingCommand(PIDComponent pidBrain, DriveSubsystem swerveSubsystem, Rotation2d desiredHeadingAllianceRelative, boolean isAllianceRelative) {
        this.pidBrain = pidBrain;
        this.swerveSubsystem = swerveSubsystem;
        desiredHeading_allianceOrField = desiredHeadingAllianceRelative;
        this.isAllianceRelative = isAllianceRelative;

        addRequirements(swerveSubsystem);
    }

    @Override public void initialize() {
        rotationalController.setPID(pidBrain.pConstant(), pidBrain.iConstant(), pidBrain.dConstant());
        rotationalController.reset();
        rotationalController.enableContinuousInput(-Math.PI, Math.PI);
        rotationalController.setTolerance(Math.PI / 90);

        heading = desiredHeading_allianceOrField;
        if (isAllianceRelative) {
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (!shouldFlip) { //TODO hack idk why this works
                heading = heading.plus(Rotation2d.fromDegrees(180));
            }
        }
    }

    @Override public void execute() {
        if (rotationalController.atSetpoint()) {
            return;
        }

        double controlOut = rotationalController
                .calculate(
                        heading.getRadians(),
                        swerveSubsystem.odometry.getHeading_fieldRelative().getRadians()
                );


        swerveSubsystem.orderToUnfiltered(new ChassisSpeeds(0,0,controlOut));
    }

    @Override public boolean isFinished() {
        return rotationalController.atSetpoint();
    }

    @Override public void end(boolean interrupted) {
        swerveSubsystem.orderToZero();
    }
}

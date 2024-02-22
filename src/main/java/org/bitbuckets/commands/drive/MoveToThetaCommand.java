package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.AutoSubsystem;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

import java.util.function.Supplier;

/**
 * Testing theta command
 */
public class MoveToThetaCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final AutoSubsystem autoSubsystem;
    final PIDComponent pidComponent;
    final double desiredHeadingTheta;

    public MoveToThetaCommand(DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, ProfiledPIDController thetaPID, PIDComponent pidComponent, double desiredHeadingTheta, AutoSubsystem autoSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.autoSubsystem = autoSubsystem;
        this.pidComponent = pidComponent;
        this.desiredHeadingTheta = desiredHeadingTheta;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double state = odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians();
        double reference = desiredHeadingTheta;

        double rotationFeedback = autoSubsystem.thetaPid_radians.calculate(state, reference);


        driveSubsystem.driveUsingChassisSpeed(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        0,
                        0,
                        rotationFeedback,
                        odometrySubsystem.getGyroAngle()
                ),
                true
        );

        pidComponent.reportState(state);
        pidComponent.reportReference(reference);

    }

    @Override
    public boolean isFinished() {
        return autoSubsystem.thetaPid_radians.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}

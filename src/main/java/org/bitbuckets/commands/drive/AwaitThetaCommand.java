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
public class AwaitThetaCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final AutoSubsystem autoSubsystem;
    final PIDComponent pidComponent;
    final Supplier<Rotation2d> desiredHeadingTheta;

    public AwaitThetaCommand(DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, AutoSubsystem autoSubsystem, PIDComponent pidComponent, Supplier<Rotation2d> desiredHeadingTheta) {
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.autoSubsystem = autoSubsystem;
        this.pidComponent = pidComponent;
        this.desiredHeadingTheta = desiredHeadingTheta;
    }

    @Override
    public void execute() {
        double state = odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians();
        double reference = desiredHeadingTheta.get().getRadians();

        double rotationFeedback = autoSubsystem.thetaPid_radians.calculate(state, reference);


        driveSubsystem.driveUsingChassisSpeed(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        0,
                        0,
                        rotationFeedback,
                        odometrySubsystem.getGyroAngle()
                )
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

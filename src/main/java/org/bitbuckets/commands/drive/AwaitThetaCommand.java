package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

/**
 * Testing theta command
 */
public class AwaitThetaCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final ProfiledPIDController thetaPID;
    final PIDComponent pidComponent;
    final double desiredHeadingTheta;

    public AwaitThetaCommand(DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, ProfiledPIDController thetaPID, PIDComponent pidComponent, double desiredHeadingTheta) {
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.thetaPID = thetaPID;
        this.pidComponent = pidComponent;
        this.desiredHeadingTheta = desiredHeadingTheta;
    }

    @Override
    public void initialize() {
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        thetaPID.setTolerance(Math.PI / 360); //0.5 deg
    }

    @Override
    public void execute() {
        double state = odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians();
        double reference = desiredHeadingTheta;

        double rotationFeedback = thetaPID.calculate(state, reference);


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

        if (Math.abs(state-reference) <0.1) {
            driveSubsystem.commandWheelsToZero();
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}

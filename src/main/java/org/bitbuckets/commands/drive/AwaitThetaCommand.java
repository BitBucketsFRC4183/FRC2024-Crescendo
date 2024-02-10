package org.bitbuckets.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

/**
 * Testing theta command
 */
public class AwaitThetaCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final ProfiledPIDController thetaPID;
    final double desiredHeadingTheta;

    public AwaitThetaCommand(DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, ProfiledPIDController thetaPID, double desiredHeadingTheta) {
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.thetaPID = thetaPID;
        this.desiredHeadingTheta = desiredHeadingTheta;
    }

    @Override
    public void initialize() {
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        thetaPID.setTolerance(Math.PI / 360); //0.5 deg
    }

    @Override
    public void execute() {
        double rotationFeedback = thetaPID.calculate(
                odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians(),
                desiredHeadingTheta
        );


        driveSubsystem.driveUsingChassisSpeed(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        0,
                        0,
                        rotationFeedback,
                        odometrySubsystem.getGyroAngle()
                )
        );
    }
}

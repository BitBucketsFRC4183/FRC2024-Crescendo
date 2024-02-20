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
public class MoveToThetaCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final ProfiledPIDController thetaPID;
    final PIDComponent pidComponent;
    final double desiredHeadingTheta;

    public MoveToThetaCommand(DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, ProfiledPIDController thetaPID, PIDComponent pidComponent, double desiredHeadingTheta) {
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.thetaPID = thetaPID;
        this.pidComponent = pidComponent;
        this.desiredHeadingTheta = desiredHeadingTheta;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        thetaPID.setTolerance(Math.PI / 90); //1 deg
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
                ),
                true
        );

        pidComponent.reportState(state);
        pidComponent.reportReference(reference);

    }

    @Override public boolean isFinished() {
        return thetaPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}
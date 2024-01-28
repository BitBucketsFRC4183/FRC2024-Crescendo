package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveComponent;

/**
 * Using SwerveMAX code to be better
 */
public class AugmentedDriveCommand extends Command {

    final OperatorInput operatorInput;
    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final SwerveComponent swerveComponent;

    public AugmentedDriveCommand(SwerveComponent swerveComponent, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, OperatorInput operatorInput) {
        this.operatorInput = operatorInput;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.swerveComponent = swerveComponent;

        addRequirements(driveSubsystem);
    }

    //copy pasted shit
    @Override
    public void execute() {

        double x = operatorInput.getRobotForwardComponentRaw();
        double y = operatorInput.getDriverRightComponentRaw();
        double theta = operatorInput.getDriverRightStickX();

        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(x, y);
        theta = MathUtil.applyDeadband(theta, 0.1);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude; //TODO slew this again
        theta = Math.copySign(theta * theta, theta);

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        linearVelocity.getX() * 3d,
                        linearVelocity.getY() * 3d,
                        theta * Math.PI / 2 );

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometrySubsystem.getGyroAngle());

        driveSubsystem.driveUsingChassisSpeed(speeds);

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

}

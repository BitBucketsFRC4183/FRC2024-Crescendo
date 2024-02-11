package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
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

        magnitudeChange = new SlewRateLimiter(swerveComponent.magnitudeFwLimit());
        addRequirements(driveSubsystem);
    }

    final SlewRateLimiter magnitudeChange;
    double lastTime = WPIUtilJNI.now() * 1e-6;

    //copy pasted shit
    @Override
    public void execute() {

        double now = WPIUtilJNI.now() * 1e-6;
        double dt = now - lastTime;

        double x = operatorInput.getRobotForwardComponentRaw();
        double y = operatorInput.getDriverRightComponentRaw();
        double theta = operatorInput.getDriverRightStickX();

        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(x, y);
        theta = MathUtil.applyDeadband(theta, 0.1);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude; //TODO slew this again
        theta = Math.copySign(theta * theta, theta);

        //TODO limiter
        //linearMagnitude = magnitudeChange.calculate(linearMagnitude);

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        linearVelocity.getX() * 10d,
                        linearVelocity.getY() * 10d,
                        theta *  2 * Math.PI );

        if (RobotContainer.SWERVE.fieldOriented()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometrySubsystem.getGyroAngle());
        }

        var desiredDeltaPose =
            new Pose2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                new Rotation2d(speeds.omegaRadiansPerSecond * dt));
        var twist = new Pose2d().log(desiredDeltaPose);
        speeds = new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt); //second order comp
        driveSubsystem.driveUsingChassisSpeed(speeds);

        lastTime = now;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

}

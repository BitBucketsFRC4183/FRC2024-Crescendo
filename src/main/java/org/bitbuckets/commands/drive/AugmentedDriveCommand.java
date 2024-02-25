package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
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
    final PIDController pidController;


    public AugmentedDriveCommand(SwerveComponent swerveComponent, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, OperatorInput operatorInput) {
        this.operatorInput = operatorInput;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.swerveComponent = swerveComponent;

        magnitudeChange = new SlewRateLimiter(swerveComponent.magnitudeFwLimit());
        pidController = new PIDController(swerveComponent.thetaModePSeed(), 0, 0);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(Math.PI / 45);
        addRequirements(driveSubsystem);
    }

    final SlewRateLimiter magnitudeChange;
    double lastTime = WPIUtilJNI.now() * 1e-6;

    //copy pasted shit
    @Override
    public void execute() {

        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;


        double now = WPIUtilJNI.now() * 1e-6;
        double dt = now - lastTime;

        double x = operatorInput.getRobotForwardComponentRaw(); //[-1, 1]
        double y = operatorInput.getDriverRightComponentRaw(); //[-1, 1]
        double theta;
        double output;



        if (operatorInput.isPIDStick()) {
            Rotation2d desiredHeading = new Rotation2d(operatorInput.getDriverRightStickY(), operatorInput.getDriverRightStickX());
            Rotation2d currentHeading = odometrySubsystem.getGyroAngle();

            theta = Math.hypot(operatorInput.getDriverRightStickY(), operatorInput.getDriverRightStickX());
            output = pidController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());
        } else {
            theta = operatorInput.getDriverRightStickX();
            output = theta * 1 * Math.PI;

        }
//        double theta = operatorInput.getDriverRightStickX();
//        double fieldY = operatorInput.getDriverRightStickX();
//        double fieldX = operatorInput.getDriverRightStickY();

        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.05);
        Rotation2d linearDirection = new Rotation2d(x, y);
        theta = MathUtil.applyDeadband(theta, 0.1);
        pidController.setP(theta * swerveComponent.thetaModePSeed());

        linearMagnitude = linearMagnitude * linearMagnitude;
        theta = Math.copySign(theta * theta, theta);

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        double speedMultiplier = 3d;
        double slowSpeedMultiplier = 1.5d;
        double turboSpeedMultiplier = 5;
        if (operatorInput.getTurboModeHeld())
        {
            speedMultiplier = turboSpeedMultiplier;
        }
        if (operatorInput.getSlowModeHeld())
        {
            speedMultiplier = slowSpeedMultiplier;
        }
        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        linearVelocity.getX() * speedMultiplier, //4.5 is the experimentally determined max velocity
                        linearVelocity.getY() * speedMultiplier,
                        output);


        if (RobotContainer.SWERVE.fieldOriented()) {
            Rotation2d gyroAngle = odometrySubsystem.getGyroAngle();
            if (shouldFlip) {
                gyroAngle = gyroAngle.plus(Rotation2d.fromDegrees(180));
            }

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,gyroAngle);
        }

        driveSubsystem.driveUsingChassisSpeed(speeds, swerveComponent.useVelocityPID());

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

}

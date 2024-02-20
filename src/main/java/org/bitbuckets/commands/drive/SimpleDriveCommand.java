package org.bitbuckets.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveComponent;

import java.util.Set;

public class SimpleDriveCommand extends Command {

    final SwerveComponent swerveComponent;
    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final OperatorInput operatorInput;

    public SimpleDriveCommand(SwerveComponent swerveComponent, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, OperatorInput operatorInput) {
        this.swerveComponent = swerveComponent;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.operatorInput = operatorInput;

        addRequirements(driveSubsystem);

    }

    @Override
    public void execute() {
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;


        double speedMultiplier = 4d;
        double slowSpeedMultiplier = 1.5;
        double turboSpeedMultiplier = 6d;
        if (operatorInput.getTurboModeHeld())
        {
            speedMultiplier = turboSpeedMultiplier;
        }
        if (operatorInput.getSlowModeHeld())
        {
            speedMultiplier = slowSpeedMultiplier;
        }
        ChassisSpeeds speeds = new ChassisSpeeds(
                speedMultiplier*operatorInput.getRobotForwardComponent(),
                speedMultiplier*operatorInput.getDriverRightComponent(),
                speedMultiplier * operatorInput.getDriverRightStickX()
        );

        if (RobotContainer.SWERVE.fieldOriented()) {
            Rotation2d gyroAngle = odometrySubsystem.getGyroAngle();
            if (shouldFlip) {
                gyroAngle = gyroAngle.plus(Rotation2d.fromDegrees(180));
            }

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,gyroAngle);
        }

        driveSubsystem.driveUsingChassisSpeed(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(driveSubsystem);
    }
}

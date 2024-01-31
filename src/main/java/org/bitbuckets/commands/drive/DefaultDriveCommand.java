package org.bitbuckets.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveComponent;
import org.bitbuckets.util.FastTrig;

import java.util.Set;

public class DefaultDriveCommand extends Command {

    final SwerveComponent swerveComponent;
    final DriveSubsystem driveSubsystem;
    final OdometrySubsystem odometrySubsystem;
    final OperatorInput operatorInput;

    public DefaultDriveCommand(SwerveComponent swerveComponent, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem, OperatorInput operatorInput) {
        this.swerveComponent = swerveComponent;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.operatorInput = operatorInput;

        addRequirements(driveSubsystem);

    }



    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
                3d*operatorInput.getRobotForwardComponent(),
                3d*operatorInput.getDriverRightComponent(),
                3 * operatorInput.getDriverRightStickX()
        );

        if (RobotContainer.SWERVE.fieldOriented()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometrySubsystem.getGyroAngle());
        }

        driveSubsystem.driveUsingChassisSpeed(speeds);
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

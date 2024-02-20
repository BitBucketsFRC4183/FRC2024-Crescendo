package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

import java.util.Optional;

/**
 * Follows a trajectory. Doesn't stop the motors once it ends and ends when the calculated trajectory time is over, doesn't leave room for PID controllers to finish.
 * You probably dont want to use this one directly
 */
public class FollowTrajectoryExactCommand extends Command {


    final Timer timer = new Timer();

    final ChoreoTrajectory trajectory;
    final OdometrySubsystem odometrySubsystem;
    final DriveSubsystem driveSubsystem;

    final PIDController xPid;
    final PIDController yPid;
    final ProfiledPIDController thetaPid;
    final boolean reZeroOdometry;

    public FollowTrajectoryExactCommand(ChoreoTrajectory trajectory, OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem, PIDController xPid, PIDController yPid, ProfiledPIDController thetaPid, boolean reZeroOdometry) {
        this.trajectory = trajectory;
        this.odometrySubsystem = odometrySubsystem;
        this.driveSubsystem = driveSubsystem;

        this.xPid = xPid;
        this.yPid = yPid;
        this.thetaPid = thetaPid;
        this.reZeroOdometry = reZeroOdometry;
    }

    boolean shouldMirror() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return alliance == DriverStation.Alliance.Red;
    }

    @Override
    public void initialize() {
        ChoreoTrajectoryState initialState =trajectory.sample(0, shouldMirror());
        if (reZeroOdometry) odometrySubsystem.forceOdometryToThinkWeAreAt(new Pose3d(initialState.getPose()));

        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.PI / 360 ); //0.5 deg
        timer.restart();
    }

    @Override
    public void execute() {

        double time = timer.get();
        ChoreoTrajectoryState trajectoryReference = trajectory.sample(time, shouldMirror());
        Pose2d robotState = odometrySubsystem.getRobotCentroidPosition();

        double xFF = trajectoryReference.velocityX;
        double yFF = trajectoryReference.velocityY;
        double rotationFF = trajectoryReference.angularVelocity;

        double xFeedback = xPid.calculate(robotState.getX(), trajectoryReference.x);
        double yFeedback = yPid.calculate(robotState.getY(), trajectoryReference.y);
        double rotationFeedback = thetaPid.calculate(robotState.getRotation().getRadians(), trajectoryReference.heading);

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                robotState.getRotation()
        );



        driveSubsystem.driveUsingChassisSpeed(robotRelativeSpeeds, true);
    }

    @Override
    public boolean isFinished() {
         //
        return (timer.hasElapsed(trajectory.getTotalTime()) && thetaPid.atSetpoint()) || timer.hasElapsed(trajectory.getTotalTime() + 2);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            driveSubsystem.driveUsingChassisSpeed(new ChassisSpeeds(), false);
        }
    }
}

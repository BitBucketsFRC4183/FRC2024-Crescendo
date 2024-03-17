package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.auto.pid.IPIDController;
import xyz.auriium.mattlib2.auto.pid.LinearPIDBrain;
import xyz.auriium.mattlib2.auto.pid.RotationalPIDBrain;

/**
 * Follows a trajectory. Doesn't stop the motors once it ends and ends when the calculated trajectory time is over, doesn't leave room for PID controllers to finish.
 * You probably dont want to use this one directly
 */
public class FollowTrajectoryExactCommand extends Command {


    final Timer timer = new Timer();

    final ChoreoTrajectory trajectory;
    final DriveSubsystem swerveSubsystem;

    final LinearPIDBrain xPidBrain;
    final LinearPIDBrain yPidBrain;
    final RotationalPIDBrain thetaPidBrain;

    IPIDController xPid;
    IPIDController yPid;
    IPIDController thetaPid;

    public FollowTrajectoryExactCommand(ChoreoTrajectory trajectory, DriveSubsystem swerveSubsystem, LinearPIDBrain xPidBrain, LinearPIDBrain yPidBrain, RotationalPIDBrain thetaPidBrain) {
        this.trajectory = trajectory;
        this.swerveSubsystem = swerveSubsystem;
        this.xPidBrain = xPidBrain;
        this.yPidBrain = yPidBrain;
        this.thetaPidBrain = thetaPidBrain;
    }

    boolean shouldMirror() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseThrow();
        return alliance == DriverStation.Alliance.Red;
    }

    //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION
    @Override
    public void initialize() {
        xPid = xPidBrain.spawn();
        yPid = yPidBrain.spawn();
        thetaPid = thetaPidBrain.spawn();

        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        ChoreoTrajectoryState trajectoryReference = trajectory.sample(time, shouldMirror());
        Pose2d robotState = swerveSubsystem.odometry.getRobotCentroidPosition();  //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION
        Rotation2d robotHeading = robotState.getRotation();

        double xFF = trajectoryReference.velocityX;
        double yFF = trajectoryReference.velocityY;
        double rotationFF = trajectoryReference.angularVelocity;

        double xFeedback = xPid.controlToReference_primeUnits(trajectoryReference.x, robotState.getX());
        double yFeedback = yPid.controlToReference_primeUnits(trajectoryReference.y, robotState.getY());
        double rotationFeedback = thetaPid.controlToReference_primeUnits(trajectoryReference.heading, robotHeading.getRadians());

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                robotState.getRotation()
        );

        swerveSubsystem.orderToUnfiltered(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {
         //
        return (timer.hasElapsed(trajectory.getTotalTime()) && thetaPid.isAtSetpoint()) || timer.hasElapsed(trajectory.getTotalTime() + 2);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            swerveSubsystem.orderToZero();
        }
    }
}

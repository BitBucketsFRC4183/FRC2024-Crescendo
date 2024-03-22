package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.auto.pid.IPIDController;
import xyz.auriium.mattlib2.auto.pid.LinearPIDBrain;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

/**
 * Follows a trajectory. Doesn't stop the motors once it ends and ends when the calculated trajectory time is over, doesn't leave room for PID controllers to finish.
 * You probably dont want to use this one directly
 */
public class FollowTrajectoryHardCommand extends Command {


    final Timer timer = new Timer();

    final ChoreoTrajectory trajectory;
    final DriveSubsystem swerveSubsystem;

    final LinearPIDBrain xPidBrain;
    final LinearPIDBrain yPidBrain;

    IPIDController xPid;
    IPIDController yPid;

    Rotation2d desiredHeading = new Rotation2d();
    double timeLast_seconds;

    public FollowTrajectoryHardCommand(ChoreoTrajectory trajectory, DriveSubsystem swerveSubsystem, LinearPIDBrain xPidBrain, LinearPIDBrain yPidBrain) {
        this.trajectory = trajectory;
        this.swerveSubsystem = swerveSubsystem;
        this.xPidBrain = xPidBrain;
        this.yPidBrain = yPidBrain;

        addRequirements(swerveSubsystem);
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
        desiredHeading = Rotation2d.fromRadians(trajectory.sample(0, shouldMirror()).heading);

        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        ChoreoTrajectoryState trajectoryReference = trajectory.sample(time, shouldMirror());
        Pose2d robotState = swerveSubsystem.odometry.getRobotCentroidPosition();  //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION

        double xFF = trajectoryReference.velocityX;
        double yFF = trajectoryReference.velocityY;
        double rotationFF = trajectoryReference.angularVelocity;

        ChoreoTrajectoryState trajectoryNext = trajectory.sample(time + 0.02, shouldMirror());
        double xNext = trajectoryNext.velocityX;
        double yNext = trajectoryNext.velocityY;
        double rotationNext = trajectoryNext.angularVelocity;

        double xFeedback = xPid.controlToReference_primeUnits(trajectoryReference.x, robotState.getX());
        double yFeedback = yPid.controlToReference_primeUnits(trajectoryReference.y, robotState.getY());

        double timeNow_seconds = MathSharedStore.getTimestamp();
        double dt_seconds = timeNow_seconds - timeLast_seconds;
        double dTheta_radianSeconds = trajectoryReference.angularVelocity;

        desiredHeading = Rotation2d.fromRadians(trajectoryReference.heading);
        desiredHeading = desiredHeading.plus(new Rotation2d(dTheta_radianSeconds * dt_seconds));
        Rotation2d errorHeading = desiredHeading.minus(swerveSubsystem.odometry.getHeading_fieldRelative());
        double feedback_u = errorHeading.getRadians() / dt_seconds * 0.16;

        if (Math.abs(errorHeading.getDegrees()) < 2) {
            feedback_u = 0;
        }


        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + feedback_u,
                robotState.getRotation()
        );

        ChassisSpeeds nextSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xNext,
                yNext,
                rotationNext,
                robotState.getRotation()
        );

        timeLast_seconds = timeNow_seconds;
        swerveSubsystem.orderToUnfilteredAuto(robotRelativeSpeeds, nextSpeeds);


    }

    @Override
    public boolean isFinished() {
         //
        return (timer.hasElapsed(trajectory.getTotalTime())); //TODO needs to be 1
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            swerveSubsystem.orderToZero();
        }
    }
}

package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
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
public class FollowTrajectoryExactCommand extends Command {


    final Timer timer = new Timer();

    final ChoreoTrajectory trajectory;
    final DriveSubsystem swerveSubsystem;

    final LinearPIDBrain xPidBrain;
    final LinearPIDBrain yPidBrain;
    final ProfiledPIDController thetaPid;

    IPIDController xPid;
    IPIDController yPid;
    final PIDComponent thetaPidComponent;

    public FollowTrajectoryExactCommand(ChoreoTrajectory trajectory, DriveSubsystem swerveSubsystem, LinearPIDBrain xPidBrain, LinearPIDBrain yPidBrain, PIDComponent thetaPidComponent) {
        this.trajectory = trajectory;
        this.swerveSubsystem = swerveSubsystem;
        this.xPidBrain = xPidBrain;
        this.yPidBrain = yPidBrain;
        this.thetaPidComponent = thetaPidComponent;

        thetaPid = new ProfiledPIDController(
                thetaPidComponent.pConstant(),
                thetaPidComponent.iConstant(),
                thetaPidComponent.dConstant(),
                new TrapezoidProfile.Constraints(3,3)
        );

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
        thetaPid.reset(MathUtil.angleModulus(swerveSubsystem.odometry.getHeading_fieldRelative().getRadians()));
        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.PI / 45); // 2 deg

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

        ChoreoTrajectoryState trajectoryNext = trajectory.sample(time + 0.02, shouldMirror());
        double xNext = trajectoryNext.velocityX;
        double yNext = trajectoryNext.velocityY;
        double rotationNext = trajectoryNext.angularVelocity;


        double xFeedback = xPid.controlToReference_primeUnits(trajectoryReference.x, robotState.getX());
        double yFeedback = yPid.controlToReference_primeUnits(trajectoryReference.y, robotState.getY());
        double rotationFeedback = thetaPid.calculate(MathUtil.angleModulus(robotHeading.getRadians()), MathUtil.angleModulus(trajectoryReference.heading));

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                robotState.getRotation()
        );

        ChassisSpeeds nextSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xNext,
                yNext,
                rotationNext,
                robotState.getRotation()
        );



        swerveSubsystem.orderToUnfilteredAuto(robotRelativeSpeeds, nextSpeeds);
    }

    @Override
    public boolean isFinished() {
         //
        return (timer.hasElapsed(trajectory.getTotalTime()) && thetaPid.atSetpoint()) || timer.hasElapsed(trajectory.getTotalTime() + 1); //TODO needs to be 1
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            swerveSubsystem.orderToZero();
        }
    }
}

package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.AutoSubsystem;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import xyz.auriium.mattlib2.utils.AngleUtil;

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
    final PIDController yPid ;
    final ProfiledPIDController thetaPid;
    final AutoSubsystem autoSubsystem;

    public FollowTrajectoryExactCommand(ChoreoTrajectory trajectory, OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem, PIDController xPid, PIDController yPid, ProfiledPIDController thetaPid, AutoSubsystem autoSubsystem) {
        this.trajectory = trajectory;
        this.odometrySubsystem = odometrySubsystem;
        this.driveSubsystem = driveSubsystem;
        this.autoSubsystem = autoSubsystem;

        this.xPid = xPid;
        this.yPid = yPid;
        this.thetaPid = thetaPid;
    }

    boolean shouldMirror() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseThrow();
        return alliance == DriverStation.Alliance.Red;
    }

    //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION
    @Override
    public void initialize() {
        thetaPid.reset(MathUtil.angleModulus(odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians()));
        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.PI / 90 ); //0.5 deg
        timer.restart();
    }

    @Override
    public void execute() {

        RobotContainer.DRIVE_T_PID.reportState(MathUtil.angleModulus(odometrySubsystem.getRobotCentroidPosition().getRotation().getDegrees()));

        double time = timer.get();
        ChoreoTrajectoryState trajectoryReference = trajectory.sample(time, shouldMirror());
        Pose2d robotState = odometrySubsystem.getRobotCentroidPosition();  //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION

        double xFF = trajectoryReference.velocityX;
        double yFF = trajectoryReference.velocityY;
        double rotationFF = trajectoryReference.angularVelocity;

        double xFeedback = xPid.calculate(robotState.getX(), trajectoryReference.x);
        double yFeedback = yPid.calculate(robotState.getY(), trajectoryReference.y);
        //TODO ITS GOTTA BE CENTROIDPOSITION.GETROTATION
        double rotationFeedback = thetaPid.calculate(MathUtil.angleModulus(odometrySubsystem.getRobotCentroidPosition().getRotation().getRadians()), MathUtil.angleModulus(trajectoryReference.heading));

        RobotContainer.DRIVE_T_PID.reportReference(MathUtil.angleModulus(MathUtil.angleModulus(trajectoryReference.heading)));


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
        return (timer.hasElapsed(trajectory.getTotalTime()) && autoSubsystem.isThetaAtSetpoint())
                || timer.hasElapsed(trajectory.getTotalTime() + 2);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            driveSubsystem.driveUsingChassisSpeed(new ChassisSpeeds(), false);
        }
    }
}

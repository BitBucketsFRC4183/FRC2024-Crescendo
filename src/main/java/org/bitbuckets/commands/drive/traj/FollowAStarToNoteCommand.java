package org.bitbuckets.commands.drive.traj;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.bitbuckets.drive.AutoSubsystem;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.util.ConversionUtil;
import org.bitbuckets.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;


import java.util.Optional;

public class FollowAStarToNoteCommand extends Command {

    final OdometrySubsystem odometrySubsystem;
    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final AutoSubsystem autoSubsystem;
    final PathConstraints pathConstraints;
    final LocalADStar adStarPlanner;

    public FollowAStarToNoteCommand(OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, AutoSubsystem autoSubsystem, PathConstraints pathConstraints, LocalADStar adStarPlanner) {
        this.odometrySubsystem = odometrySubsystem;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.autoSubsystem = autoSubsystem;
        this.pathConstraints = pathConstraints;
        this.adStarPlanner = adStarPlanner;

        addRequirements(driveSubsystem, autoSubsystem);
    }

    PathPlannerTrajectory cachedTrajectory;

    final Timer totalTrajectoryTimer = new Timer();
    final Timer currentTrajectoryTimer = new Timer();

    double killTime = 0;
    Pose2d lastGuarunteedPose = new Pose2d();

    @Override
    public void initialize() {
        Optional<Pose2d> output = visionSubsystem.getClosestNotePose();
        lastGuarunteedPose = output.get();

        adStarPlanner.setStartPosition( odometrySubsystem.getRobotCentroidPosition().getTranslation() );
        adStarPlanner.setGoalPosition( lastGuarunteedPose.getTranslation() );


        var pp = adStarPlanner
                .getCurrentPath(
                        pathConstraints,
                        new GoalEndState(0, new Rotation2d(0), false)
                );

        if (pp != null) {
            cachedTrajectory = pp.getTrajectory(
                    odometrySubsystem.robotVelocity_metersPerSecond(),
                    odometrySubsystem.getGyroAngle()
            );
        } SmartDashboard.putNumberArray("traj", new double[0]);

        if (cachedTrajectory != null) {
            Pose2d[] trajForLog = cachedTrajectory
                    .getStates()
                    .stream()
                    .map(PathPlannerTrajectory.State::getTargetHolonomicPose)
                    .toArray(Pose2d[]::new);


            SmartDashboard.putNumberArray("traj", ConversionUtil.fromPoseArray(trajForLog));
            killTime = cachedTrajectory.getTotalTimeSeconds();
        }

        totalTrajectoryTimer.restart();
        currentTrajectoryTimer.restart();
    }

    @Override
    public void execute() {
        /*visionSubsystem.getClosestNotePose().ifPresent(pos -> {
            //take delta if it has changed

            boolean deltaXSignificant = Math.abs(pos.getX() - lastGuarunteedPose.getX()) > 0.1;
            boolean deltaYSignificant = Math.abs(pos.getY() - lastGuarunteedPose.getY()) > 0.1;
            if (deltaXSignificant || deltaYSignificant) {
                adStarPlanner.setGoalPosition(pos.getTranslation());
            }
        });*/




        if (adStarPlanner.isNewPathAvailable()) {
            currentTrajectoryTimer.restart();

            var ooga = adStarPlanner
                    .getCurrentPath(pathConstraints, new GoalEndState(0, new Rotation2d(0), false));

            if (ooga == null) return;

            cachedTrajectory = ooga.getTrajectory(
                            odometrySubsystem.robotVelocity_metersPerSecond(),
                            odometrySubsystem.getGyroAngle()
                    );

            Pose2d[] trajForLog = cachedTrajectory
                    .getStates()
                    .stream()
                    .map(PathPlannerTrajectory.State::getTargetHolonomicPose)
                    .toArray(Pose2d[]::new);

            SmartDashboard.putNumberArray("traj", ConversionUtil.fromPoseArray(trajForLog));

            killTime += cachedTrajectory.getTotalTimeSeconds();
        }

        if (cachedTrajectory != null) {
            var state = cachedTrajectory.sample(currentTrajectoryTimer.get());

            var translationalVelocities = new Pose2d(new Translation2d(), state.heading)
                    .transformBy(new Transform2d(state.velocityMps, 0, new Rotation2d()))
                    .getTranslation();

            var rotationalVelocity = state.headingAngularVelocityRps;

            ChassisSpeeds feedBackSpeeds = autoSubsystem.calculateFeedbackSpeeds(state.getTargetHolonomicPose());
            ChassisSpeeds feedforwardSpeeds_fieldRelative = new ChassisSpeeds(
                    translationalVelocities.getX() + feedBackSpeeds.vxMetersPerSecond,
                    translationalVelocities.getY() + feedBackSpeeds.vyMetersPerSecond,
                    rotationalVelocity + feedBackSpeeds.omegaRadiansPerSecond
            );

            if (feedforwardSpeeds_fieldRelative.omegaRadiansPerSecond > 1) {
                //System.out.println("DOING THING");
            }
            ChassisSpeeds robotRelativeFeedForward = ChassisSpeeds.fromFieldRelativeSpeeds(feedforwardSpeeds_fieldRelative, odometrySubsystem.getGyroAngle());
            //robotRelativeFeedForward = ChassisSpeeds.discretize(robotRelativeFeedForward, 0.02);



            driveSubsystem.driveUsingChassisSpeed(robotRelativeFeedForward, false);
        }


    }

    @Override
    public boolean isFinished() {


        boolean hasExceededTime = totalTrajectoryTimer.hasElapsed(12); //It is taking too long

        if (cachedTrajectory == null) return hasExceededTime;

        boolean hasFinishedLocal = currentTrajectoryTimer.hasElapsed(cachedTrajectory.getTotalTimeSeconds());


        return hasExceededTime || hasFinishedLocal;
    }

    @Override public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}

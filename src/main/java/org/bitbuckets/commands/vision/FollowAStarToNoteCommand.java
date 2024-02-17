package org.bitbuckets.commands.vision;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;


import java.util.Optional;

public class FollowAStarToNoteCommand extends Command {

    final OdometrySubsystem odometrySubsystem;
    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final PathConstraints pathConstraints;
    final LocalADStar adStarPlanner;

    public FollowAStarToNoteCommand(OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PathConstraints pathConstraints, LocalADStar adStarPlanner) {
        this.odometrySubsystem = odometrySubsystem;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pathConstraints = pathConstraints;
        this.adStarPlanner = adStarPlanner;
    }

    PathPlannerTrajectory cachedTrajectory;

    final Timer timer = new Timer();

    @Override
    public void initialize() {
        Optional<Pose2d> output = visionSubsystem.getClosestNotePose();
        Pose2d guarunteedPose = output.get();

        adStarPlanner.setStartPosition( odometrySubsystem.getRobotCentroidPosition().getTranslation() );
        adStarPlanner.setGoalPosition( guarunteedPose.getTranslation() );

        cachedTrajectory = adStarPlanner
                .getCurrentPath(
                        pathConstraints,
                        new GoalEndState(0, new Rotation2d(0), false)
                )
                .getTrajectory(
                        odometrySubsystem.robotVelocity_metersPerSecond(),
                        odometrySubsystem.getGyroAngle()
                );

        timer.restart();
    }

    @Override
    public void execute() {
        if (adStarPlanner.isNewPathAvailable()) {
            cachedTrajectory = adStarPlanner
                    .getCurrentPath(pathConstraints, new GoalEndState(0, new Rotation2d(0), false))
                    .getTrajectory(
                            odometrySubsystem.robotVelocity_metersPerSecond(),
                            odometrySubsystem.getGyroAngle()
                    );
        }

        cachedTrajectory.sample(timer.get());
/*

        var t = cachedPath.getTrajectory();
*/


        //adStarPlanner.setGoalPosition(new Translation2d());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

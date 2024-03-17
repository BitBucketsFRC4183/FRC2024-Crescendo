package org.bitbuckets.commands.drive.odo;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.Odometry;

public class PlaceOdometryCommand extends Command {


    final ChoreoTrajectory trajectory;
    final Odometry odometry;

    public PlaceOdometryCommand(ChoreoTrajectory trajectory, Odometry odometry) {
        this.trajectory = trajectory;
        this.odometry = odometry;
    }

    boolean shouldMirror() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseThrow();
        return alliance == DriverStation.Alliance.Red;
    }

    @Override
    public void initialize() {
        Pose2d initialPose = trajectory.getInitialPose();

        if (shouldMirror()) {
            initialPose = trajectory.flipped().getInitialPose();
        }

        odometry.forcePosition(initialPose);
    }

    @Override public boolean isFinished() {
        return true;
    }
}

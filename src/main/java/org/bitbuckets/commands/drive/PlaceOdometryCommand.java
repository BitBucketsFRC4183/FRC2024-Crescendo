package org.bitbuckets.commands.drive;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.OdometrySubsystem;

public class PlaceOdometryCommand extends Command {


    final ChoreoTrajectory trajectory;
    final OdometrySubsystem odometrySubsystem;

    public PlaceOdometryCommand(ChoreoTrajectory trajectory, OdometrySubsystem odometrySubsystem) {
        this.trajectory = trajectory;
        this.odometrySubsystem = odometrySubsystem;
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

        System.out.println("Initial Pose: " + initialPose.toString() + "flipped: " + shouldMirror());

        odometrySubsystem.forceOdometryToThinkWeAreAt(new Pose3d(initialPose));
    }
}

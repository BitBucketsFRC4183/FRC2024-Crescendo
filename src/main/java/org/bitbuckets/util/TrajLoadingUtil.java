package org.bitbuckets.util;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.google.gson.Gson;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

public class TrajLoadingUtil {

    static final Gson GSON = new Gson();

    public static ChoreoTrajectory getTrajectory(String choreoRoutine, String part) {
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        var routine_dir = new File(traj_dir, choreoRoutine);
        var traj_file = new File(routine_dir, part + ".traj");

        return loadFile(traj_file);
    }

    public static ChoreoTrajectory loadFile(File path) {
        try {
            var reader = new BufferedReader(new FileReader(path));
            ChoreoTrajectory traj = GSON.fromJson(reader, ChoreoTrajectory.class);

            return traj;
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }


    public static ChoreoControlFunction choreoSwerveController(
            PIDController xController, PIDController yController, ProfiledPIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback =
                    rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
        };
    }
}

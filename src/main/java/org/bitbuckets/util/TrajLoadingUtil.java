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
import java.util.ArrayList;
import java.util.List;

public class TrajLoadingUtil {

    static final Gson GSON = new Gson();

    public static ChoreoTrajectory[] getAllTrajectories(String choreoPath) {
        var choreo_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        int i = 1; //starting index to look at

        List<ChoreoTrajectory> choreoTrajList = new ArrayList<>();


        while (true) {
            File toCheck = new File(choreo_dir, choreoPath + "." + i + ".traj");

            if (!toCheck.exists()) break;

            //it's there
            i++;
            choreoTrajList.add(loadFile(toCheck));
        }
        System.out.println("done: " + choreoPath);
        return choreoTrajList.toArray(ChoreoTrajectory[]::new);



        //let's look for files until we can't find any
    }


    public static ChoreoTrajectory loadFile(File path) {
        try {
            var reader = new BufferedReader(new FileReader(path));

            return GSON.fromJson(reader, ChoreoTrajectory.class);
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }


}

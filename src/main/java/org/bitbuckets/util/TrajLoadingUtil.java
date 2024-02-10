package org.bitbuckets.util;

import com.choreo.lib.ChoreoTrajectory;
import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

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


}

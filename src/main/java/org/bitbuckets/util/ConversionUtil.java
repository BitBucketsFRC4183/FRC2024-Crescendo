package org.bitbuckets.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ConversionUtil {

    public static double[] fromPoseArray(Pose2d... trajForLog) {


        double[] actualForLog = new double[trajForLog.length * 3];

        for (int i = 0; i < trajForLog.length; i++) {
            actualForLog[i * 3] = trajForLog[i].getX();
            actualForLog[i * 3 + 1] = trajForLog[i].getY();
            actualForLog[i * 3 + 2] = trajForLog[i].getRotation().getRadians();
        }

        return actualForLog;
    }

}

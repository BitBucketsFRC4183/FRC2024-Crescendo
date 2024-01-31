package org.bitbuckets.util;

import org.bitbuckets.Robot;

public class CachedRobot {

    static boolean isReal = Robot.isReal();

    public static boolean isReal() {
        return isReal;
    }

}

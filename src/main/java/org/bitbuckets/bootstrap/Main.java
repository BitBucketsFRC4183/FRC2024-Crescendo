package org.bitbuckets.bootstrap;

import edu.wpi.first.wpilibj.CachingRobotBase;
import org.bitbuckets.Robot;
import xyz.auriium.yuukonstants.exception.ExceptionUtil;

public class Main {

    public static void main(String[] args) {
        CachingRobotBase.startRobot(ExceptionUtil.wrapExceptionalSupplier(() -> new CheckedRobot(new Robot())));
    }

}

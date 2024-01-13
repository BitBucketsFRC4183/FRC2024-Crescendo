package org.bitbuckets.bootstrap;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import org.bitbuckets.Robot;
import yuukonstants.exception.ExceptionUtil;

public class Main {

    public static void main(String[] args) {
        RobotBase.startRobot(ExceptionUtil.wrapExceptionalSupplier(() -> new CheckedRobot(new Robot())));
    }

}

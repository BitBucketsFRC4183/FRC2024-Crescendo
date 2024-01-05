package org.bitbuckets.bootstrap;

import edu.wpi.first.wpilibj.RobotBase;
import org.bitbuckets.Robot;

public class Main {

    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }

}

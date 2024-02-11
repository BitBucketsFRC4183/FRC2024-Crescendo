package org.bitbuckets.util;

import edu.wpi.first.wpilibj.DigitalInput;
import xyz.auriium.mattlib2.MattlibSettings;

public class WhichRobotUtil {

    public static MattlibSettings.Robot loadRobot() {
        try (DigitalInput dio = new DigitalInput(9)) { //we physically plug a dio signal to ground short into the dio port 9 to say "this rio is mcr"
            return !dio.get() ? MattlibSettings.Robot.MCR : MattlibSettings.Robot.CARY;
        }
    }


}

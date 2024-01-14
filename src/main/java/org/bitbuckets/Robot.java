package org.bitbuckets;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.bitbuckets.util.ThriftyEncoder;
import xyz.auriium.mattlib2.Mattlib;

public class Robot extends TimedRobot {



    RobotContainer container;

    @Override
    public void robotInit() {
        container = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
        Mattlib.LOOPER.runPeriodicLoop();

        CommandScheduler.getInstance().run();
    }

}

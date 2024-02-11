package org.bitbuckets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import xyz.auriium.mattlib2.Mattlib;

public class Robot extends TimedRobot {


    RobotContainer container;
    double simKillerCounter;

    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);

        container = new RobotContainer();
        if (isSimulation()) {
            System.out.println("we in the matrix baby");
        }
    }

    @Override
    public void robotPeriodic() {
        Mattlib.LOOPER.runPeriodicLoop();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        container.autonomousInit();
    }

    @Override
    public void disabledInit() {
        container.disabledInit();
    }

    @Override
    public void teleopInit() {
        container.teleopInit();
    }

    @Override
    public void testInit() {
        container.testInit();
    }

    @Override
    public void simulationPeriodic() {
        if (System.getenv().containsKey("CI")) {
            if (++simKillerCounter >= 300) {
                System.exit(0);
            }
        }
        container.simulationPeriodic();

    }


}

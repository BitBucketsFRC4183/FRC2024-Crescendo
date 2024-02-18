package org.bitbuckets.bootstrap;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.CachingTimedRobot;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.TimedRobot;
import xyz.auriium.yuukonstants.exception.ExceptionUtil;

import java.util.Optional;

/**
 * Forces all functions through exceptionutil
 */
public class CheckedRobot extends CachingTimedRobot {


    final CachingTimedRobot delegate;

    public CheckedRobot(CachingTimedRobot delegate) {
        this.delegate = delegate;
    }

    @Override
    public void robotInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::robotInit).run();
    }

    @Override
    public void robotPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::robotPeriodic).run();
    }


    @Override
    public void simulationInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::simulationInit).run();
    }

    @Override
    public void disabledInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::disabledInit).run();
    }

    @Override
    public void autonomousInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::autonomousInit).run();
    }

    @Override
    public void teleopInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::teleopInit).run();
    }

    @Override
    public void testInit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::testInit).run();
    }

    @Override
    public void simulationPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::simulationPeriodic).run();
    }

    @Override
    public void disabledPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::disabledPeriodic).run();
    }

    @Override
    public void autonomousPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::autonomousPeriodic).run();
    }

    @Override
    public void teleopPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::teleopPeriodic).run();
    }

    @Override
    public void testPeriodic() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::testPeriodic).run();
    }

    @Override
    public void disabledExit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::disabledExit).run();
    }

    @Override
    public void autonomousExit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::autonomousExit).run();
    }

    @Override
    public void teleopExit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::teleopExit).run();
    }

    @Override
    public void testExit() {
        ExceptionUtil.wrapExceptionalRunnable(delegate::testExit).run();
    }
}

package org.bitbuckets;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.commands.drive.DefaultDriveCommand;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;
import yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class OperatorInput implements IPeriodicLooped {

    final CommandXboxController driver = new CommandXboxController(0);
    final Trigger isTeleop = null; //TODO fill this out

    final DriveSubsystem driveSubsystem;

    final Trigger slowModeHold = driver.leftTrigger();
    final Trigger turboModeHold = driver.rightTrigger();
    final Trigger autoAlignHold = driver.a();
    final Trigger xButtonToggle = driver.x();
    final Trigger groundIntakeHold = driver.rightBumper();
    final Trigger resetGyroToggle = driver.start();
    double driverLeftStickX, driverLeftStickY, driverRightStickX, driverRightStickY;


    public OperatorInput(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        mattRegister();
    }

    @Override
    public Optional<ExplainedException> verifyInit() {

        DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(driveSubsystem, this);

        //When driver
        Trigger xGreaterThan = driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1);
        Trigger yGreaterThan = driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
        Trigger rotGreaterThan = driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1);

        isTeleop.and(xGreaterThan.or(yGreaterThan).or(rotGreaterThan)).whileTrue(defaultDriveCommand);



        return Optional.empty();
    }

    public boolean getSlowModeState() {
        return slowModeHold.getAsBoolean();
    }

    public boolean getTurboModeState() {
        return turboModeHold.getAsBoolean();
    }

    public boolean getAutoAlignState() {
        return autoAlignHold.getAsBoolean();
    }

    public boolean getXButtonState() {
        return xButtonToggle.getAsBoolean();
    }

    public boolean getGroundIntakeState() {
        return groundIntakeHold.getAsBoolean();
    }

    public boolean getResetGyroState() {
        return resetGyroToggle.getAsBoolean();
    }

    public double getDriverLeftStickX() {
        return driverLeftStickX;
    }

    public double getDriverLeftStickY() {
        return driverLeftStickY;
    }

    public double getDriverRightStickX() {
        return driverRightStickX;
    }

    public double getDriverRightStickY() {
        return driverRightStickY;
    }

}

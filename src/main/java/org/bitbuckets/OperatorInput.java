package org.bitbuckets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class holds all of our operator triggers
 */
public class OperatorInput {
    // axis 0 and axis 1 correspond to up and down
    // axis 4 correspond to rot

    final CommandXboxController operatorControl = new CommandXboxController(1);
    final CommandXboxController driver = new CommandXboxController(0);


    public boolean actuallyIsTeleop = false;
    final Trigger isTeleop = new Trigger(() -> actuallyIsTeleop); //TODO fill this out

    final Trigger shootByVision = operatorControl.a();

    final Trigger shootManually = operatorControl.x();
    final Trigger sourceIntake_hold = operatorControl.leftBumper();
    final Trigger ampSetpoint_hold = operatorControl.leftTrigger();
    final Trigger speakerSetpoint_hold = operatorControl.rightTrigger();
    final Trigger ampVisionPriority_toggle = operatorControl.povLeft();
    final Trigger speakerVisionPriority_toggle = operatorControl.povRight();

    final Trigger setShooterAngleManually = operatorControl.leftStick();



    final Trigger slowModeHold = driver.leftTrigger();
    final Trigger turboModeHold = driver.rightTrigger();
    final Trigger autoAlignHold = driver.x();
    final Trigger xButtonToggle = driver.a();
    final Trigger groundIntakeHold = driver.rightBumper();
    final Trigger groundOuttakeHold = driver.leftBumper();
    final Trigger groundIntakeHoldOp = operatorControl.rightBumper();
    final Trigger groundOuttakeHoldOp = operatorControl.b();
    final Trigger resetGyroPress = driver.start();


    /**
     * @param input a value
     * @return that value, deadbanded
     */
    static double deadband(double input) {
        double value = input;

        value = MathUtil.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);


        return value;
    }

    public double getClimberInput()
    {
        return deadband(operatorControl.getRawAxis(XboxController.Axis.kRightY.value));
    }





    public boolean getSlowModeState() {
        return slowModeHold.getAsBoolean();
    }

    public boolean getTurboModeHeld() {
        return turboModeHold.getAsBoolean();
    }

    public boolean getAutoAlignState() {
        return autoAlignHold.getAsBoolean();
    }

    public boolean getOperatorXButtonState() {
        return shootManually.getAsBoolean();
    }


    public boolean getDriverXButtonState() {
        return xButtonToggle.getAsBoolean();
    }

    public boolean getGroundIntakeState() {
        return groundIntakeHold.getAsBoolean();
    }

    public boolean getResetGyroState() {
        return resetGyroPress.getAsBoolean();
    }

    public double getDriverRightComponent() {
        return deadband(-driver.getLeftX()); //reference frame stuff
    }

    public double getRobotForwardComponent() {
        return deadband(-driver.getLeftY()); //reference frame stuff
    }

    public double getDriverRightComponentRaw() {
        return -driver.getLeftX(); //reference frame stuff
    }

    public double getRobotForwardComponentRaw() {
        return -driver.getLeftY(); //reference frame stuff
    }

    public double getRobotRotationRaw() {
        return driver.getRightX();
    }


    public double getDriverRightStickX() {
        return deadband(-driver.getRightX());
    }

    public double getOperatorLeftStickY(){return deadband(operatorControl.getRawAxis(XboxController.Axis.kLeftY.value));}

}

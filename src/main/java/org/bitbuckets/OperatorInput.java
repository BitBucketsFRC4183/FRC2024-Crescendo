package org.bitbuckets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class holds all of our operator triggers
 */
public class OperatorInput {
    // axis 0 and axis 1 correspond to up and down
    // axis 4 correspond to rot

    final CommandXboxController operatorControl = new CommandXboxController(1);
    public final CommandXboxController driver = new CommandXboxController(0);

    public boolean actuallyIsTeleop = false;
    final Trigger isTeleop = new Trigger(() -> actuallyIsTeleop); //TODO fill this out

    //OPERATOR'S CONTROLS
    final Trigger shootByVision = operatorControl.a();
    final Trigger shootManually = operatorControl.x();
    final Trigger sourceIntake_hold = operatorControl.leftBumper();
    final Trigger groundIntakeNoBeamBreak = operatorControl.rightTrigger();
    final Trigger ampShotSpeed = operatorControl.y();
    final Trigger groundIntakeHoldOp = operatorControl.rightBumper();
    final Trigger groundOuttakeHoldOp = operatorControl.b();
    final Trigger ampVisionPriority_toggle = operatorControl.povLeft();
    final Trigger speakerVisionPriority_toggle = operatorControl.povRight();
    final Trigger setShooterAngleManually = operatorControl.leftStick();
    final Trigger rev = operatorControl.leftTrigger();
    Trigger climberThreshold = operatorControl.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1).or(operatorControl.axisLessThan(XboxController.Axis.kRightY.value, -0.1));


    //DRIVER'S CONTROLS
    public final Trigger slowModeHold = driver.leftTrigger();
    public final Trigger turboModeHold = driver.rightTrigger();
    public final Trigger leftSpeakerHeadingHold = driver.x();
    public final Trigger frontSpeakerHeadingHold = driver.y();
    public final Trigger rightSpeakerHeadingHold = driver.b();
    public final Trigger ampHeadingHold = driver.a();

    public final Trigger customHeadingDesired = leftSpeakerHeadingHold.or(frontSpeakerHeadingHold).or(rightSpeakerHeadingHold).or(ampHeadingHold);
    public final Trigger customHeadingNotDesired = customHeadingDesired.negate();

    final Trigger resetGyroPress = driver.start();

    final Trigger xNotDesired = driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1).or(driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1)).negate();
    final Trigger yNotDesired = driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).or(driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)).negate();
    final Trigger thetaNotDesired = driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).or(driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1)).negate();
    public final Trigger movementNotDesired = xNotDesired.and(yNotDesired).and(thetaNotDesired);
    public final Trigger movementDesired = movementNotDesired.negate();


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

    public double getDriverRightComponentRaw() {
        return -driver.getLeftX(); //reference frame stuff
    }

    public double getRobotForwardComponentRaw() {
        return -driver.getLeftY(); //reference frame stuff
    }

    public double getRobotRotationRaw() {
        return driver.getRightX();
    }


    public double getDriverAngularComponentRaw() {
        return deadband(-driver.getRightX());
    }

    public double getOperatorLeftStickY(){return deadband(-operatorControl.getRawAxis(XboxController.Axis.kLeftY.value));}

    public void setOperatorVibrating(boolean shouldVibrate) {
        if (shouldVibrate) {
            operatorControl.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);

        } else {
            operatorControl.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

        }
    }



}

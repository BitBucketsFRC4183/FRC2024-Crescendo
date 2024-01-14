package org.bitbuckets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.commands.drive.DefaultDriveCommand;
import org.bitbuckets.commands.shooter.DefaultShooterCommand;
import org.bitbuckets.commands.shooter.IntakeCommand;
import org.bitbuckets.commands.shooter.SetAmpShootingAngleCommand;
import org.bitbuckets.commands.shooter.SetSpeakerShootingAngleCommand;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;
import yuukonstants.exception.ExplainedException;

import java.util.Optional;

/**
 * This class holds all of our operator triggers
 */
public class OperatorInput {

    final CommandXboxController operatorControl = new CommandXboxController(1);
    final CommandXboxController driver = new CommandXboxController(0);

    final Trigger isTeleop = null; //TODO fill this out
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
    final Trigger autoAlignHold = driver.a();
    final Trigger xButtonToggle = driver.x();
    final Trigger groundIntakeHold = driver.rightBumper();
    final Trigger resetGyroToggle = driver.start();


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

    public boolean getTurboModeState() {
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
        return resetGyroToggle.getAsBoolean();
    }

    public double getDriverLeftStickX() {
        return operatorControl.getLeftX();
    }

    public double getOperatorLeftStickY(){return operatorControl.getLeftY();}

}

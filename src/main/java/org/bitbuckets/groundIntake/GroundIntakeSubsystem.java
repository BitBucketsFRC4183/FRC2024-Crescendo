package org.bitbuckets.groundIntake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class GroundIntakeSubsystem implements Subsystem, IPeriodicLooped {

    public final ILinearController topMotor;
    public final ILinearController bottomMotor;
    public final SimpleMotorFeedforward feedForward;


    public GroundIntakeSubsystem(ILinearController topMotor, ILinearController bottomMotor, SimpleMotorFeedforward feedForward) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.feedForward = feedForward;

        register();
        mattRegister();
    }

    // copypasted from ClimberSubsystem
    public void setGroundIntakeSpeeds(double topMotorSpeed_metersPerSecond, double bottomMotorSpeed_metersPerSecond) {
        double leftVoltage = feedForward.calculate(topMotorSpeed_metersPerSecond);
        double rightVoltage = feedForward.calculate(bottomMotorSpeed_metersPerSecond);

        topMotor.setToVoltage(leftVoltage);
        topMotor.setToVoltage(rightVoltage);

        bottomMotor.linearVelocity_mechanismMetersPerSecond();
        bottomMotor.linearVelocity_mechanismMetersPerSecond();
    }
    public void setToVoltage(double voltage) {
        topMotor.setToVoltage(voltage);
        bottomMotor.setToVoltage(voltage); // TODO add inversion to bottom in conf
    }

    public void setMotorsZero(){
        topMotor.setToVoltage(0);
        bottomMotor.setToVoltage(0);
    }



}

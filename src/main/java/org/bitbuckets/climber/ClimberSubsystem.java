package org.bitbuckets.climber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearController;

public class ClimberSubsystem {

    final ILinearController leftMotor;
    final ILinearController rightMotor;
    final SimpleMotorFeedforward feedforward;

    public ClimberSubsystem(ILinearController leftMotor, ILinearController rightMotor, SimpleMotorFeedforward feedforward) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.feedforward = feedforward;
    }

    public void setFFElevatorSpeeds(double leftMotorSpeed_metersPerSecond, double rightMotorSpeed_metersPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_metersPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_metersPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);

        leftMotor.linearVelocity_mechanismMetersPerSecond();
        rightMotor.linearVelocity_mechanismMetersPerSecond();


    }



    public void setToVoltage(double voltage)
    {
        leftMotor.setToVoltage(voltage);
        rightMotor.setToVoltage(voltage);
    }

    public void setMotorsZero() {
        leftMotor.setToVoltage(0);
        rightMotor.setToVoltage(0);
    }







}

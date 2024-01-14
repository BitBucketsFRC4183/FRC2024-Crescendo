package org.bitbuckets.climber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class ClimberSubsystem {

    final ILinearMotor leftMotor;
    final ILinearMotor rightMotor;
    final ILinearController motorController;
    final SimpleMotorFeedforward feedforward;

    public ClimberSubsystem(ILinearMotor leftMotor, ILinearMotor rightMotor, ILinearController motorController, SimpleMotorFeedforward feedforward) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.motorController = motorController;
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


    public void moveToPosition(double setpointMechanism_meters) {
        motorController.controlToLinearReference(setpointMechanism_meters);
    }





}

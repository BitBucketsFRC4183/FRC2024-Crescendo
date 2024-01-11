package org.bitbuckets.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class ShooterSubsystem implements Subsystem {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4,3);
    final IRotationalMotor leftMotor1Wheel;
    final IRotationalMotor rightMotor1Wheel;
    final IRotationalMotor leftMotor2Wheel;
    final IRotationalMotor rightMotor2Wheel;

    public ShooterSubsystem(IRotationalMotor leftMotor1Wheel, IRotationalMotor rightMotor1Wheel, IRotationalMotor leftMotor2Wheel, IRotationalMotor rightMotor2Wheel) {
        this.leftMotor1Wheel = leftMotor1Wheel;
        this.rightMotor1Wheel = rightMotor1Wheel;
        this.leftMotor2Wheel = leftMotor2Wheel;
        this.rightMotor2Wheel = rightMotor2Wheel;
                
        register();
    }

    public void setMotorSpeeds(double leftMotorSpeed1Wheel, double rightMotorSpeed1Wheel, double leftMotorSpeed2Wheel, double rightMotorSpeed2Wheel)
    {

        double leftVoltage1Wheel = feedforward.calculate(leftMotorSpeed1Wheel);
        double rightVoltage1Wheel = feedforward.calculate(rightMotorSpeed1Wheel);
        double leftVoltage2Wheel = feedforward.calculate(leftMotorSpeed2Wheel);
        double rightVoltage2Wheel = feedforward.calculate(rightMotorSpeed2Wheel);

        leftMotor1Wheel.setToVoltage(leftVoltage1Wheel);
        rightMotor1Wheel.setToVoltage(rightVoltage1Wheel);
        leftMotor2Wheel.setToVoltage(leftVoltage2Wheel);
        rightMotor2Wheel.setToVoltage(rightVoltage2Wheel);
    }


    @Override
    public void periodic() {

    }

}

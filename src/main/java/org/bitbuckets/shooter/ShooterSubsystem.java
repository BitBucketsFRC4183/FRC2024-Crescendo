package org.bitbuckets.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class ShooterSubsystem implements Subsystem {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4,3);
    final IRotationalMotor leftMotor;
    final IRotationalMotor rightMotor;

    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        register();
    }

    public void setMotorSpeeds(double leftMotorSpeed, double rightMotorSpeed)
    {

        double leftVoltage = feedforward.calculate(leftMotorSpeed);
        double rightVoltage = feedforward.calculate(rightMotorSpeed);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);
    }


    @Override
    public void periodic() {

    }

}

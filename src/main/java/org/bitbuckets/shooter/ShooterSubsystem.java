package org.bitbuckets.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.OperatorInput;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class ShooterSubsystem implements Subsystem {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4, 3);
    final IRotationalMotor leftMotor;
    final IRotationalMotor rightMotor;
    final OperatorInput operatorInput;


    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor, ILinearMotor pivotMotor, OperatorInput operatorInput) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.operatorInput = operatorInput;

        register();
    }

    public void setMotorRotationalSpeeds(double leftMotorSpeed1Wheel_rotationsPerSecond, double rightMotorSpeed1Wheel_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed1Wheel_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed1Wheel_rotationsPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);


        leftMotor.angularVelocity_mechanismRotationsPerSecond();
        rightMotor.angularVelocity_mechanismRotationsPerSecond();
    }

    public double rotationSpeedToLinearSpeed(double rotationalSpeed, double radius) {
        return rotationalSpeed * radius;
    }

    public void convertWheelSpeedToNoteSpeed(double motorSpeed) {


    }


    @Override
    public void periodic() {

    }

}

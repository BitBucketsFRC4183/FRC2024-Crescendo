package org.bitbuckets.noteManagement;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class NoteManagementSubsystem implements Subsystem {

    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4, 3);
    final ILinearController motorOne;
    final ILinearMotor motorTwo;


    public NoteManagementSubsystem(IRotationalMotor motorOne, IRotationalMotor motorTwo, IRotationEncoder absoluteEncoder) {
        this.motorOne = (ILinearController) motorOne;
        this.motorTwo = (ILinearMotor) motorTwo;
    }

    @Override
    public void periodic() {

    }

    public void setMotorRotationalSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);

        motorOne.setToVoltage(leftVoltage);
        motorTwo.setToVoltage(rightVoltage);

    }

    public void setAllMotorsToVoltage(double voltage) {
        motorOne.setToVoltage(voltage);
        motorTwo.setToVoltage(voltage);
    }

    public void maintainSpeed(double motorOne_rotationsPerSecond, double motorTwo_rotationsPerSecond) {
        setMotorRotationalSpeeds(motorOne_rotationsPerSecond, motorTwo_rotationsPerSecond);
    }
}

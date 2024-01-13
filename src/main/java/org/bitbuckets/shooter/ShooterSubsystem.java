package org.bitbuckets.shooter;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;
import xyz.auriium.mattlib2.hardware.IRotationalPositionControl;
import yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class ShooterSubsystem implements Subsystem, IPeriodicLooped {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4,3);
    final IRotationalMotor leftMotor;
    final IRotationalMotor rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder absoluteEncoder;
    final ShooterComponent shooterComponent;

    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor, IRotationalController angleMotor, IRotationEncoder absoluteEncoder, ShooterComponent shooterComponent) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.shooterComponent = shooterComponent;

        mattRegister();
        register();
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        absoluteEncoder.forceRotationalOffset(
                shooterComponent.absEncoderOffset()
        );

        angleMotor.forceRotationalOffset(
                absoluteEncoder.angularPosition_normalizedMechanismRotations()
        );

        return Optional.empty();
    }


    public void setMotorRotationalSpeeds(double leftMotorSpeed1Wheel_rotationsPerSecond, double rightMotorSpeed1Wheel_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed1Wheel_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed1Wheel_rotationsPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);

        leftMotor.angularVelocity_mechanismRotationsPerSecond();
        rightMotor.angularVelocity_mechanismRotationsPerSecond();
    }

    public void setAllMotorsToVoltage(double voltage)
    {
        leftMotor.setToVoltage(voltage);
        rightMotor.setToVoltage(voltage);
    }

    public void moveToAngle(double angle_degrees) {
        double mechanism_rotations = angle_degrees/360d;
        angleMotor.controlToNormalizedReference(mechanism_rotations);
    }

    public void moveToMechanismPosition(double mechanism_positions)
    {
        angleMotor.controlToNormalizedReference(mechanism_positions);
    }

    public void setZeroAngle()
    {

        double offset = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        angleMotor.forceRotationalOffset(offset);
    }

    public void intake() {
        moveToAngle(45);
        // rotate wheels in the other direction
        setMotorRotationalSpeeds(0,0);
    }

    @Override
    public void periodic() {

    }

}

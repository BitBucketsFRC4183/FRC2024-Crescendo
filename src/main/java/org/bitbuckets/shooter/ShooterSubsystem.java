package org.bitbuckets.shooter;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.AbsoluteEncoderComponent;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class ShooterSubsystem implements Subsystem, IMattlibHooked {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward;
    public final IRotationalController leftMotor; //TODO find a way to not use public here (linearFFGenRoutine)
    public final IRotationalController rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder absoluteEncoder;
    final ShooterComponent shooterComponent;
    final AbsoluteEncoderComponent encoderComponent;
    final IRotationEncoder velocityEncoder;

    public ShooterSubsystem(IRotationalController leftMotor, IRotationalController rightMotor, IRotationalController angleMotor, IRotationEncoder absoluteEncoder, ShooterComponent shooterComponent, AbsoluteEncoderComponent encoderComponent, IRotationEncoder velocityEncoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.shooterComponent = shooterComponent;
        this.encoderComponent = encoderComponent;
        this.velocityEncoder = velocityEncoder;

        feedforward = new SimpleMotorFeedforward(shooterComponent.ks(),shooterComponent.kv());

        mattRegister();
        register();
    }

    @Override
    public ExplainedException[] verifyInit() {
        absoluteEncoder.forceRotationalOffset(
                encoderComponent.offset_mechanismRotations()
        );

        angleMotor.forceRotationalOffset(
                absoluteEncoder.angularPosition_normalizedMechanismRotations()
        );

        return new ExplainedException[0];
    }


    @Override
    public void periodic() {
        //if motor at limit switch, rezero



    }


    public void setMotorRotationalSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);
    }

    public void setAllMotorsToVoltage(double voltage) {
        leftMotor.setToVoltage(voltage);
        rightMotor.setToVoltage(voltage);
    }

    public void moveToRotation(double mechanism_rotations) {
        angleMotor.controlToNormalizedReference(mechanism_rotations);

    }

    public void setPivotMotorToVoltage(double voltage) {
        angleMotor.setToVoltage(voltage);
    }

    public void setZeroAngle() {

        double offset = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        angleMotor.forceRotationalOffset(offset);
    }


    public boolean hasReachedSpeeds(double leftSpeeds, double rightSpeeds) {
        boolean leftAtSpeed = leftMotor.angularVelocity_mechanismRotationsPerSecond() >= leftSpeeds;
        boolean rightAtSpeed = rightMotor.angularVelocity_mechanismRotationsPerSecond() >= rightSpeeds;

        return leftAtSpeed && rightAtSpeed;
    }

    public static boolean isWithinDeadband(double deadband, double target, double actual) {
        return actual >= target - deadband || actual <= target + deadband;
    }

    public boolean hasReachedAngle(double angle_mechanismRotations) {
        double currentPos_mechRot = angleMotor.angularPosition_normalizedMechanismRotations();

        return isWithinDeadband(shooterComponent.deadband_mechanismRotations(), angle_mechanismRotations, currentPos_mechRot);
    }

    public double getPivotAnglePosition_normalizedMechanismRotations() {
        return angleMotor.angularPosition_normalizedMechanismRotations();
    }

    public double calculateMinimalAngleForSpeaker() {
        return 0;
    }
    public double calculateMaximalAngleForSpeaker() {
        return 0.16472536666;
    }




}

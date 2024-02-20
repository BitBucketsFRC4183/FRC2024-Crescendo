package org.bitbuckets.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

public class PivotSubsystem implements Subsystem, IMattlibHooked {

    final IRotationalController leftAngleMotor;
    final IRotationalController rightAngleMotor;
    final IRotationEncoder pivotEncoder;

    public PivotSubsystem(IRotationalController leftAngleMotor, IRotationalController rightAngleMotor, IRotationEncoder pivotEncoder) {
        this.leftAngleMotor = leftAngleMotor;
        this.rightAngleMotor = rightAngleMotor;
        this.pivotEncoder = pivotEncoder;

        mattRegister();
        register();
    }

    public static boolean isWithinDeadband(double deadband, double target, double actual) {
        return actual >= target - deadband || actual <= target + deadband;
    }

    @Override public ExplainedException[] verifyInit() {
        leftAngleMotor.forceRotationalOffset(
                pivotEncoder.angularPosition_normalizedMechanismRotations()
        );

        return new ExplainedException[0];
    }


    public void moveToRotation(double mechanism_rotations) {
        leftAngleMotor.controlToNormalizedReference(mechanism_rotations);

    }

    public void setPivotMotorToVoltage(double voltage) {
        leftAngleMotor.setToVoltage(voltage);
        rightAngleMotor.setToVoltage(voltage);
    }

    public void setZeroAngle() {
        double offset = pivotEncoder.angularPosition_normalizedMechanismRotations();
        leftAngleMotor.forceRotationalOffset(offset);
    }

    public boolean hasReachedAngle(double angle_mechanismRotations) {
        double currentPos_mechRot = leftAngleMotor.angularPosition_normalizedMechanismRotations();

        return isWithinDeadband(0.5 , angle_mechanismRotations, currentPos_mechRot); //TODO
    }

    public double getPivotAnglePosition_normalizedMechanismRotations() {
        return leftAngleMotor.angularPosition_normalizedMechanismRotations();
    }


}

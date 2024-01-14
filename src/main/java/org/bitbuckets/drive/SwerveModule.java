package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;

public class SwerveModule implements IPeriodicLooped {

    final ILinearMotor driveMotor;
    final IRotationalController steerController;
    final IRotationEncoder absoluteEncoder;

    public SwerveModule(ILinearMotor driveMotor, IRotationalController steerController, IRotationEncoder absoluteEncoder) {
        this.driveMotor = driveMotor;
        this.steerController = steerController;
        this.absoluteEncoder = absoluteEncoder;

        mattRegister();
    }

    //periodic looped stuff & state

    int resetIteration = 0;

    /**
     * Resets the relative encoder using the absolute encoder if you are still for long enough
     */
    @Override
    public void logicPeriodic() {
        if (steerController.angularVelocity_mechanismRotationsPerSecond() * 10 >= 0.5) return;
        if (++resetIteration > 500) {
            resetIteration = 0;
            double absoluteAngularPosition_infiniteMechanismRotations = absoluteEncoder.angularPosition_normalizedMechanismRotations();
            steerController.forceRotationalOffset(absoluteAngularPosition_infiniteMechanismRotations);
        }
    }

    //implementation stuff

    /**
     * Commands all the motors to immediately stop moving. If this is not called, motors commanded with set will
     * move until eternity
     */
    public void stopAllMotors() {
        driveMotor.setToVoltage(0);
        steerController.setToVoltage(0);
    }

    /**
     *
     * @param driveVoltage a voltage to drive at until eternity
     * @param referenceAngle_normalizedMechanismRotations a rotation to PID to until eternity
     */
    public void setToMoveAt(double driveVoltage, double referenceAngle_normalizedMechanismRotations) {

        //referenceAngle is 0 to 1, currentSteerAngle is 0 to 1
        double difference = referenceAngle_normalizedMechanismRotations - currentSteerAngle_normalizedMechanismRotations();

        //we want difference to be -0.5 to 0.5
        if (difference >= 0.5) { //let's say it's .7
            referenceAngle_normalizedMechanismRotations -= 1.0; //now it's -.3
        } else if (difference < -0.5) { //-.7
            referenceAngle_normalizedMechanismRotations += 1.0; //.3
        }

        difference = referenceAngle_normalizedMechanismRotations - currentSteerAngle_normalizedMechanismRotations(); // Recalculate difference

        // If the difference is greater than 0.25 or less than 0.25 the drive can be inverted so the total
        // movement of the module is less than 0.25
        if (difference > 0.25 || difference < -0.25) {
            // Only need to add 180 deg here because the target angle will be put back into the range 0 to 1
            referenceAngle_normalizedMechanismRotations += 0.5;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range 0 to 1
        referenceAngle_normalizedMechanismRotations %= (1.0);
        if (referenceAngle_normalizedMechanismRotations < 0.0) {
            referenceAngle_normalizedMechanismRotations += 1.0;
        }

        steerController.controlToNormalizedReference(
                referenceAngle_normalizedMechanismRotations
        );

        driveMotor.setToVoltage(driveVoltage);
    }


    //getters

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.linearPosition_mechanismMeters(),
                Rotation2d.fromRotations(
                        absoluteEncoder.angularPosition_normalizedMechanismRotations()
                )
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.linearVelocity_mechanismMetersPerSecond(),
                Rotation2d.fromRotations(
                        absoluteEncoder.angularPosition_normalizedMechanismRotations()
                )
        );
    }

    public double getDriveVelocity_metersPerSecond() {
        return driveMotor.linearVelocity_mechanismMetersPerSecond();
    }

    public double currentSteerAngle_normalizedMechanismRotations() {
        return absoluteEncoder.angularPosition_normalizedMechanismRotations();
    }



}

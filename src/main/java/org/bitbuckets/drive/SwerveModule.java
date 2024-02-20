package org.bitbuckets.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.bitbuckets.util.Util;
import xyz.auriium.mattlib2.hardware.ILinearVelocityController;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

public class SwerveModule implements IMattlibHooked {

    public final ILinearVelocityController driveMotor;
    final IRotationalController steerController;
    final IRotationEncoder absoluteEncoder;
    final SimpleMotorFeedforward ff;

    public SwerveModule(ILinearVelocityController driveMotor, IRotationalController steerController, IRotationEncoder absoluteEncoder, SimpleMotorFeedforward ff) {
        this.driveMotor = driveMotor;
        this.steerController = steerController;
        this.absoluteEncoder = absoluteEncoder;
        this.ff = ff;

        mattRegister();
    }

    //periodic looped stuff & state

    /**
     * Resets this swerve module using the absolute encoder's idea of zero
     */
    public void resetToAbsolute() {
        double absoluteAngularPosition_infiniteMechanismRotations = absoluteEncoder.angularPosition_mechanismRotations();
        steerController.forceRotationalOffset(absoluteAngularPosition_infiniteMechanismRotations);

    }

    @Override
    public ExplainedException[] verifyInit() {
        resetToAbsolute();
        return new ExplainedException[0];
    }

    int resetIteration = 0;

    /**
     * Resets the relative encoder using the absolute encoder if you are still for long enough
     */
    @Override
    public void logicPeriodic() {
        if (steerController.angularVelocity_mechanismRotationsPerSecond() * 10 >= 0.5) {
            resetIteration = 0;
            return;
        }
        if (++resetIteration > 500) {
            resetIteration = 0;
            resetToAbsolute();
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

    public void setToMoveAt(SwerveModuleState state, boolean usePID) {

       //SwerveModuleState optimizedState = state;



        SwerveModuleState optimizedState = SwerveModuleState.optimize(
                state,
                Rotation2d.fromRotations(steerController.angularPosition_normalizedMechanismRotations())
        );

        if (optimizedState.speedMetersPerSecond < 0.001 && Math.abs(optimizedState.angle.getRotations() - steerController.angularPosition_normalizedMechanismRotations()) < 0.01) {
            steerController.setToVoltage(0);
            driveMotor.setToVoltage(0);
        }

        steerController.controlToNormalizedReference(
                optimizedState.angle.getRotations()
        );

        double feedforwardVoltage = ff.calculate(optimizedState.speedMetersPerSecond);
        feedforwardVoltage = MathUtil.clamp(feedforwardVoltage, -Util.MAX_VOLTAGE, Util.MAX_VOLTAGE);

        if (usePID) {
            driveMotor.controlToLinearVelocityReferenceArbitrary(optimizedState.speedMetersPerSecond, feedforwardVoltage);
        } else {
            driveMotor.setToVoltage(feedforwardVoltage);
        }
    }


    //getters

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.linearPosition_mechanismMeters(),
                Rotation2d.fromRotations(
                        steerController.angularPosition_normalizedMechanismRotations()
                )
        );
    }

    public SwerveModuleState getHallEffectBasedState() {
        return new SwerveModuleState(
                driveMotor.linearVelocity_mechanismMetersPerSecond(),
                Rotation2d.fromRotations(
                        steerController.angularPosition_normalizedMechanismRotations()
                )
        );
    }

    public SwerveModuleState getAbsoluteBasedState() {
        return new SwerveModuleState(
                driveMotor.linearVelocity_mechanismMetersPerSecond(),
                Rotation2d.fromRotations(
                        absoluteEncoder.angularPosition_normalizedMechanismRotations()
                )
        );
    }



}

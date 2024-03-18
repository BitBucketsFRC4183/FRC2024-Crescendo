package org.bitbuckets.drive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
    final SimpleMotorFeedforward driveFF;
    final LinearPlantInversionFeedforward<N1, N1, N1> augmentedFFPlant;


    public SwerveModule(ILinearVelocityController driveMotor, IRotationalController steerController, IRotationEncoder absoluteEncoder, SimpleMotorFeedforward driveFF) {
        this.driveMotor = driveMotor;
        this.steerController = steerController;
        this.absoluteEncoder = absoluteEncoder;
        this.driveFF = driveFF;
        this.augmentedFFPlant = new LinearPlantInversionFeedforward<>(LinearSystemId.identifyVelocitySystem(driveFF.kv, driveFF.ka), 0.02);

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
/*
    double calculateFF(double currentVelocity, double nextVelocity) {
        var r = MatBuilder.fill(Nat.N1(), Nat.N1(), currentVelocity);
        var nextR = MatBuilder.fill(Nat.N1(), Nat.N1(), nextVelocity);

        return driveFF.ks * Math.signum(currentVelocity) + augmentedFFPlant.calculate(r, nextR).get(0, 0);
    }*/

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
        driveMotor.stopActuator();
        steerController.stopActuator();
    }

    public void setToHeading(SwerveModuleState optimizedState) {
        if (optimizedState.speedMetersPerSecond < 0.001 && Math.abs(optimizedState.angle.getRotations() - steerController.angularPosition_normalizedMechanismRotations()) < 0.01) {
            steerController.setToVoltage(0);
            driveMotor.setToVoltage(0);
        }

        steerController.controlToNormalizedReference(
                optimizedState.angle.getRotations()
        );
    }

    public void setToMoveAtFuture(SwerveModuleState now, SwerveModuleState future) {

        steerController.controlToNormalizedReference(
                now.angle.getRotations()
        );


        var r = MatBuilder.fill(Nat.N1(), Nat.N1(), now.speedMetersPerSecond);
        var nextR = MatBuilder.fill(Nat.N1(), Nat.N1(), future.speedMetersPerSecond);

        double staticFF =  driveFF.ks * Math.signum(now.speedMetersPerSecond);
        double velocityFF = augmentedFFPlant.calculate(r, nextR).get(0,0);

        double feedforwardVoltage = MathUtil.clamp(staticFF + velocityFF, -Util.MAX_VOLTAGE, Util.MAX_VOLTAGE);


        driveMotor.controlToLinearVelocityReferenceArbitrary(now.speedMetersPerSecond, feedforwardVoltage);

    }

    public void setToMoveAt(SwerveModuleState optimizedState, boolean usePID) {

        if (optimizedState.speedMetersPerSecond < 0.001 && Math.abs(optimizedState.angle.getRotations() - steerController.angularPosition_normalizedMechanismRotations()) < 0.01) {
            steerController.setToVoltage(0);
            driveMotor.setToVoltage(0);
        }

        steerController.controlToNormalizedReference(
                optimizedState.angle.getRotations()
        );

        double feedforwardVoltage = driveFF.calculate(optimizedState.speedMetersPerSecond);
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

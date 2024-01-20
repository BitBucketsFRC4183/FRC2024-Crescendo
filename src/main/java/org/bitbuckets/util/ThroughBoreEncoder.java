package org.bitbuckets.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import net.bytebuddy.implementation.bytecode.Throw;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.utils.AngleUtil;


public class ThroughBoreEncoder implements IRotationEncoder {

    RelativeEncoder sparkRelativeEncoder;
    DigitalInput digitalInput;
    EncoderComponent encoderComponent;



    public ThroughBoreEncoder(RelativeEncoder sparkRelativeEncoder, EncoderComponent encoderComponent) {
       this.sparkRelativeEncoder = sparkRelativeEncoder;
       this.encoderComponent = encoderComponent;

    }

    @Override
    public void forceRotationalOffset(double v) {
    }

    @Override
    public double angularPosition_encoderRotations() {
        throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularPosition_mechanismRotations() instead!");
    }

    @Override
    public double angularPosition_mechanismRotations() {
        return sparkRelativeEncoder.getPosition();
    }

    @Override
    public double angularPosition_normalizedMechanismRotations() {
        return AngleUtil.normalizeRotations(angularPosition_normalizedMechanismRotations());
    }

    @Override
    public double angularPosition_normalizedEncoderRotations() {
        throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularPosition_normaliezdMechanismRotations() instead!");
    }

    @Override
    public double angularVelocity_mechanismRotationsPerSecond() {
        return sparkRelativeEncoder.getVelocity();
    }

    @Override
    public double angularVelocity_encoderRotationsPerSecond() {
        throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularVelocity_mechanismRotationsPerSecond() instead!");
    }
}

package org.bitbuckets.shooter;

import org.bitbuckets.commands.shooter.InterpolatingTreeMap;
import org.bitbuckets.util.AbsoluteEncoderComponent;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;

public class ShooterCalculationTest {

    IRotationalMotor leftMotor;
    IRotationalMotor rightMotor;
    IRotationalController angleMotor;
    IRotationEncoder absoluteEncoder;
    ShooterComponent shooterComponent;
    AbsoluteEncoderComponent encoderComponent;
    IRotationEncoder velocityEncoder;
    InterpolatingTreeMap speedTreeMap;

    ShooterSubsystem shooterSubsystem;

    @BeforeEach
    public void beforeEach() {
         leftMotor = mock(IRotationalMotor.class);
         rightMotor = mock(IRotationalMotor.class);
         angleMotor = mock(IRotationalController.class);
         absoluteEncoder = mock(IRotationEncoder.class);
         shooterComponent = mock(ShooterComponent.class);
         encoderComponent = mock(AbsoluteEncoderComponent.class);
         velocityEncoder = mock(IRotationEncoder.class);
         speedTreeMap = mock(InterpolatingTreeMap.class);
         shooterSubsystem = new ShooterSubsystem(leftMotor, rightMotor, angleMotor, absoluteEncoder, shooterComponent, encoderComponent, velocityEncoder, speedTreeMap);
    }

    @Test
    public void testShooterOptimalAngle_degrees() {
        assertEquals(46.5, shooterSubsystem.getOptimalAngle_degrees(35, 52.6316));
    }

    @Test
    public void testShooterOptimalVelocity_inchesPerSecond() {
        assertEquals(217.6, shooterSubsystem.getOptimalVelocity_inchesPerSecond(35, 52.6316));
    }
}

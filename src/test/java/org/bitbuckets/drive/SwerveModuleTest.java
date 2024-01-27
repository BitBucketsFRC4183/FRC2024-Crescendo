package org.bitbuckets.drive;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

public class SwerveModuleTest {

    ILinearMotor driveMotor;
    IRotationalController steerController;
    IRotationEncoder absoluteEncoder;

    @BeforeEach
    public void beforeEach() {
        driveMotor = mock(ILinearMotor.class);
        steerController = mock(IRotationalController.class);
        absoluteEncoder = mock(IRotationEncoder.class);
    }

    @Test
    public void TestLogicPeriodic() {

        IRotationalMotor steerController = mock(IRotationalMotor.class);
        when(steerController.angularVelocity_mechanismRotationsPerSecond()).thenReturn(20d);

        IRotationEncoder absoluteEncoder = mock(IRotationEncoder.class);
        when(absoluteEncoder.angularPosition_normalizedMechanismRotations()).thenReturn(10d);
        double absoluteAngularPosition_infiniteMechanismRotations = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        System.out.print(steerController.angularPosition_mechanismRotations());
        steerController.forceRotationalOffset(absoluteAngularPosition_infiniteMechanismRotations);

        assertEquals(0d, steerController.angularPosition_mechanismRotations(), 0);
    }

    @Test
    public void testSetToMoveAtStraight() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder);

        module.setToMoveAt(12, 0);
        verify(driveMotor).setToVoltage(12);
        verify(steerController).controlToNormalizedReference(0);
    }

    @Test
    public void testSetToMoveAtTurn1() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder);

        // move a quarter
        module.setToMoveAt(12, .25);
        verify(steerController).controlToNormalizedReference(.25);
    }

    @Test
    public void testSetToMoveAtTurn2() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder);

        // move a quarter
        module.setToMoveAt(12, -.25);
        verify(steerController).controlToNormalizedReference(.75);
    }

    @Test
    public void testSetToMoveAtTurn3() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder);

        // abs encoder registers 1/2 turn clockwise
        when(absoluteEncoder.angularPosition_normalizedMechanismRotations()).thenReturn(.5);

        // rotate a bit counter-clockwise
        // the "asked for" rotation would be -.7, but we flip the drive motor in reverse and only
        // actually rotate .3
        module.setToMoveAt(12, -.2);

        // the drive motor should move backwards, and the mechanism should be set to .4 (-.1 + .5)
        verify(driveMotor).setToVoltage(-12);
        verify(steerController).controlToNormalizedReference(AdditionalMatchers.eq(.3, .01));
    }

    @Test
    public void testSetToMoveAtTurn4() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder);

        // abs encoder registers 1/4 turn clockwise
        when(absoluteEncoder.angularPosition_normalizedMechanismRotations()).thenReturn(.25);

        // rotate a bit counter-clockwise
        module.setToMoveAt(12, -.1);

        // the drive motor should move backwards, and the mechanism should be set to .4 (-.1 + .5)
        verify(driveMotor).setToVoltage(-12);
        verify(steerController).controlToNormalizedReference(.4);
    }
}

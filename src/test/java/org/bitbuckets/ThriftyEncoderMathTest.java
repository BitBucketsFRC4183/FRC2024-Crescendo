package org.bitbuckets;

import org.bitbuckets.util.EncoderComponent;
import org.bitbuckets.util.ThriftyEncoder;
import org.bitbuckets.util.Util;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;


public class ThriftyEncoderMathTest {

    ThriftyEncoder thriftyEncoder;

    public ThriftyEncoderMathTest(ThriftyEncoder thriftyEncoder) {
        this.thriftyEncoder = thriftyEncoder;
    }

    @Disabled
    @Test
    public void checkThriftyAngularPosition_encoderRotations()
    {
        assertEquals(0, thriftyEncoder.angularPosition_encoderRotations(), 0.1);
    }
    
    @Disabled
    @Test
    public void checkThriftyAngularPosition_normalizedEncoderRotations()
    {
        assertEquals(0, thriftyEncoder.angularPosition_normalizedEncoderRotations(), 0.1);
    }

    @Disabled
    @Test
    public void checkThriftyAngularPosition_mechanismRotations()
    {
        assertEquals(0, thriftyEncoder.angularPosition_mechanismRotations(), 0.1);
    }

    @Disabled
    @Test
    public void checkThriftyAngularPosition_NormalizedMechanismRotations()
    {
        assertEquals(0, thriftyEncoder.angularPosition_normalizedMechanismRotations(), 0.1);
    }
    



}

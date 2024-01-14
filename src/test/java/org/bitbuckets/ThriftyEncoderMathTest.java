package org.bitbuckets;

import edu.wpi.first.wpilibj.AnalogInput;
import org.bitbuckets.util.EncoderComponent;
import org.bitbuckets.util.ThriftyEncoder;
import org.bitbuckets.util.Util;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class ThriftyEncoderMathTest {

    
    @Test
    public void checkThriftyAngularPosition_encoderRotations()
    {
        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(Mockito.mock(AnalogInput.class), Mockito.mock(EncoderComponent.class));

        assertEquals(0, thriftyEncoder.angularPosition_encoderRotations(), 0.1);
    }
    
    
    @Test
    public void checkThriftyAngularPosition_normalizedEncoderRotations()
    {
        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(Mockito.mock(AnalogInput.class), Mockito.mock(EncoderComponent.class));        assertEquals(0, thriftyEncoder.angularPosition_normalizedEncoderRotations(), 0.1);
    }

    
    @Test
    public void checkThriftyAngularPosition_mechanismRotations()
    {
        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(Mockito.mock(AnalogInput.class), Mockito.mock(EncoderComponent.class));
        assertEquals(0, thriftyEncoder.angularPosition_mechanismRotations(), 0.1);
    }

    
    @Test
    public void checkThriftyAngularPosition_NormalizedMechanismRotations()
    {
        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(Mockito.mock(AnalogInput.class), Mockito.mock(EncoderComponent.class));
        assertEquals(0, thriftyEncoder.angularPosition_normalizedMechanismRotations(), 0.1);
    }
    



}

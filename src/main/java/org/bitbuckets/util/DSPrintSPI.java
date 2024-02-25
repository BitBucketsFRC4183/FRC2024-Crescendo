package org.bitbuckets.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.kordamp.jipsy.annotations.ServiceProviderFor;
import xyz.auriium.yuukonstants.exception.ExceptionEatingSPI;
import xyz.auriium.yuukonstants.exception.ExplainedException;

@ServiceProviderFor(ExceptionEatingSPI.class)
public class DSPrintSPI implements ExceptionEatingSPI {
    @Override public void exceptionHandlingFunction(ExplainedException e) {
        DriverStation.reportError(e.toOutput(),true);
    }
}

package org.bitbuckets.groundIntake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class GroundIntakeSubsystem {

    final ILinearMotor topMotor;
    final ILinearMotor bottomMotor;
    final SimpleMotorFeedforward feedforward;


    public GroundIntakeSubsystem(ILinearMotor topMotor, ILinearMotor bottomMotor, SimpleMotorFeedforward feedforward) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.feedforward = feedforward;
    }


}

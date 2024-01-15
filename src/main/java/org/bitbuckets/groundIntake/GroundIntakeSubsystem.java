package org.bitbuckets.groundIntake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class GroundIntakeSubsystem {

    final ILinearMotor topMotor;
    final ILinearMotor bottomMotor;


    public GroundIntakeSubsystem(ILinearMotor topMotor, ILinearMotor bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void setToVoltage(double voltage) {
        topMotor.setToVoltage(voltage / 2);
        bottomMotor.setToVoltage(voltage); // TODO add inversion to bottom in conf
    }

    public void setMotorsZero(){
        topMotor.setToVoltage(0);
        bottomMotor.setToVoltage(0);
    }


}

package org.bitbuckets.groundIntake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class GroundIntakeSubsystem implements Subsystem, IPeriodicLooped {

    public final ILinearController topMotor;
    public final ILinearController bottomMotor;


    public GroundIntakeSubsystem(ILinearController topMotor, ILinearController bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;

        register();
        mattRegister();
    }

    public void setToVoltage(double voltage) {
        topMotor.setToVoltage(voltage);
        bottomMotor.setToVoltage(voltage); // TODO add inversion to bottom in conf
    }

    public void setMotorsZero(){
        topMotor.setToVoltage(0);
        bottomMotor.setToVoltage(0);
    }



}

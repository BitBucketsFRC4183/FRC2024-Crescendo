package org.bitbuckets.noteManagement;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class NoteManagementSubsystem implements Subsystem {

    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4, 3);
    final ILinearMotor nms_motor;
    final DigitalInput digitalInput;


    public NoteManagementSubsystem(ILinearMotor nms_motor, DigitalInput digitalInput) {
        this.nms_motor = nms_motor;
        this.digitalInput = digitalInput;
    }

    @Override
    public void periodic() {

    }

    public void setAllMotorsToVoltage(double nmsMotorSpeed_metersPerSecond) {
        double voltage = feedforward.calculate(nmsMotorSpeed_metersPerSecond);
        nms_motor.setToVoltage(voltage);
    }

   public boolean isBeamBreakTrue() {
       return digitalInput.get();
   }


}

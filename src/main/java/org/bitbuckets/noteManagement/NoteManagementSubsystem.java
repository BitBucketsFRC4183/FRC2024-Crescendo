package org.bitbuckets.noteManagement;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class NoteManagementSubsystem implements Subsystem {

    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4, 3);
    final ILinearMotor nms_bottomMotor;
    final ILinearMotor nms_topMotor;
    final DigitalInput digitalInput;


    public NoteManagementSubsystem(ILinearMotor nms_bottomMotor, ILinearMotor nms_topMotor, DigitalInput digitalInput) {
        this.nms_bottomMotor = nms_bottomMotor;
        this.nms_topMotor = nms_topMotor;
        this.digitalInput = digitalInput;
    }

    @Override
    public void periodic() {

    }

    public void setAllMotorsToVoltage(double bottomMotorSpeed_metersPerSecond, double topMotorSpeed_metersPerSecond) {
        double bottomVoltage = feedforward.calculate(bottomMotorSpeed_metersPerSecond);
        double topVoltage = feedforward.calculate(topMotorSpeed_metersPerSecond);
        nms_bottomMotor.setToVoltage(bottomVoltage);
        nms_topMotor.setToVoltage(topVoltage);
    }

   public boolean isNoteIn() {
       return !digitalInput.get();
   }




}

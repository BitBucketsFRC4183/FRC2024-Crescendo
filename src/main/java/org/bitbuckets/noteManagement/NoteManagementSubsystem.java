package org.bitbuckets.noteManagement;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class NoteManagementSubsystem implements Subsystem {

    final ILinearMotor nms_bottomMotor;
    final ILinearMotor nms_topMotor;
    final DigitalInput digitalInput;


    public NoteManagementSubsystem(ILinearMotor nms_bottomMotor, ILinearMotor nms_topMotor, DigitalInput digitalInput) {
        this.nms_bottomMotor = nms_bottomMotor;
        this.nms_topMotor = nms_topMotor;
        this.digitalInput = digitalInput;
    }

    public void runMotors() {
        nms_bottomMotor.setToVoltage(12);
        nms_topMotor.setToVoltage(12);
    }

    public boolean isNoteIn() {
        return !digitalInput.get();
    }




}

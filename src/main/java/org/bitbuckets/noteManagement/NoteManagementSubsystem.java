package org.bitbuckets.noteManagement;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.RunningAverageBuffer;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class NoteManagementSubsystem implements Subsystem, IMattlibHooked {

    final ILinearMotor nms_bottomMotor;
    final ILinearMotor nms_topMotor;
    final DigitalInput digitalInput;
    final NoteManagementComponent nmsComponent;
    final RunningAverageBuffer bottomBuffer;
    final RunningAverageBuffer topBuffer;
    final SimpleMotorFeedforward simpleMotorFeedforward;


    public NoteManagementSubsystem(ILinearMotor nms_bottomMotor, ILinearMotor nms_topMotor, DigitalInput digitalInput, NoteManagementComponent nmsComponent) {
        this.nms_bottomMotor = nms_bottomMotor;
        this.nms_topMotor = nms_topMotor;
        this.digitalInput = digitalInput;
        this.nmsComponent = nmsComponent;

        bottomBuffer = new RunningAverageBuffer(3);
        topBuffer = new RunningAverageBuffer(3);
        simpleMotorFeedforward = new SimpleMotorFeedforward(1, 2);



        mattRegister();
    }

    @Override
    public void logPeriodic() {

        nmsComponent.reportDIOOutput(!digitalInput.get());
    }

    public void setAllToVoltage(double voltage) {
        nms_bottomMotor.setToVoltage(voltage);
        nms_topMotor.setToVoltage(voltage);
    }

    public void setMotorSpeeds(double topMotor_metersPerSecond, double bottomMotor_metersPerSecond) {
        double topVoltage = simpleMotorFeedforward.calculate(topMotor_metersPerSecond);
        double bottomVoltage = simpleMotorFeedforward.calculate(bottomMotor_metersPerSecond);
        nms_topMotor.setToVoltage(topVoltage);
        nms_bottomMotor.setToVoltage(bottomVoltage);
    }

    public boolean isNoteIn() {
        return !digitalInput.get();
    }


    public boolean motorsAtVelocity(double exit_velocity) {
        bottomBuffer.isAtAverage(nms_bottomMotor.linearVelocity_mechanismMetersPerSecond(), exit_velocity);
        topBuffer.isAtAverage(nms_topMotor.linearVelocity_mechanismMetersPerSecond(), exit_velocity);
        throw new UnsupportedOperationException("TODO"); //TODO
    }


}

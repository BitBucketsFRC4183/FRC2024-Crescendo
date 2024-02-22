package org.bitbuckets.shooter;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.RunningAverageBuffer;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class FlywheelSubsystem implements Subsystem, IMattlibHooked {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward;
    public final IRotationalController leftMotor; //TODO find a way to not use public here (linearFFGenRoutine)
    public final IRotationalController rightMotor;

    final IRotationEncoder velocityEncoderLeft;
    final IRotationEncoder velocityEncoderRight;
    final ShooterComponent shooterComponent;

    final RunningAverageBuffer atSpeeds = new RunningAverageBuffer(4);

    public FlywheelSubsystem(IRotationalController leftMotor, IRotationalController rightMotor, ShooterComponent shooterComponent, IRotationEncoder velocityEncoderLeft, IRotationEncoder velocityEncoderRight) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.shooterComponent = shooterComponent;
        this.velocityEncoderLeft = velocityEncoderLeft;

        feedforward = new SimpleMotorFeedforward(shooterComponent.ks(),shooterComponent.kv());
        this.velocityEncoderRight = velocityEncoderRight;

        mattRegister();
        register();
    }

    public interface ShooterComponent extends INetworkedComponent {
        @Conf("ff_ks") double ks();
        @Conf("ff_kv") double kv();

        @Log("isReachedSpeeds") void reportReachedSpeeds(boolean reachedSpeeds);
    }

    boolean reachedSpeeds = false;

    @Override public void logicPeriodic() {
    }

    @Override public void logPeriodic() {
        shooterComponent.reportReachedSpeeds(reachedSpeeds);
    }

    public void setFlywheelSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);
    }

    public void setFlywheelVoltage(double voltage) {
        leftMotor.setToVoltage(voltage);
        rightMotor.setToVoltage(voltage);
    }

    public void setFlywheelToZero() {
        leftMotor.setToVoltage(0);
        rightMotor.setToVoltage(0);
    }

    public boolean hasReachedSpeeds(double leftSpeeds_mechanismRotationsPerSecond, double rightSpeeds_mechanismRotationsPerSecond) {
        boolean leftAtSpeed = Math.abs(velocityEncoderLeft.angularVelocity_mechanismRotationsPerSecond()) >= leftSpeeds_mechanismRotationsPerSecond;
        boolean rightAtSpeed = Math.abs(velocityEncoderRight.angularVelocity_mechanismRotationsPerSecond()) >= rightSpeeds_mechanismRotationsPerSecond;

        this.reachedSpeeds = leftAtSpeed && rightAtSpeed;
        return leftAtSpeed && rightAtSpeed;
    }


}

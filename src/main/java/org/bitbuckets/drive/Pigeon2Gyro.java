package org.bitbuckets.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

/**
 * MOUNTING/REFERENCE
 *
 * 0,0 is FL, right is positive x and twords back is positive y, gyro is at 5,9
 *
 */
public class Pigeon2Gyro implements IGyro, IMattlibHooked{

    final Pigeon2 pigeon2;

    StatusSignal<Double> yawCachedSignal;

    public Pigeon2Gyro(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;

        mattRegister();
    }

    @Override
    public ExplainedException[] verifyInit() {
        yawCachedSignal = pigeon2.getYaw();

        return new ExplainedException[0];
    }


    Rotation2d tare = new Rotation2d();

    @Override
    public Rotation2d initializationRelativeRotation() {
        return pigeon2.getRotation2d();
    }

    @Override
    public Rotation2d userZeroRelativeRotation() {
        return pigeon2.getRotation2d().minus(tare);
    }

    @Override
    public void userZero() {
        tare = tare.plus(userZeroRelativeRotation());
    }

    @Override
    public void userForceOffset(Rotation2d beAt) {
        tare = tare.plus(userZeroRelativeRotation()).plus(beAt);
    }

    @Override
    public boolean isOk() {

        return BaseStatusSignal.isAllGood(yawCachedSignal);
    }


}

package org.bitbuckets.drive;

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
public class Pigeon2Gyro implements IGyro, IMattlibHooked {

    final Pigeon2 pigeon2;

    public Pigeon2Gyro(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;

        mattRegister();
    }

    @Override public ExplainedException[] verifyInit() {
        //pigeon2.setYaw(0); //TODO DO NOT DO THIS PLEASE OH MY GOD DO NOT DO THIS
        pigeon2.optimizeBusUtilization();

        return IMattlibHooked.super.verifyInit();
    }

    @Override
    public Rotation2d rotation_initializationRelative() {

        return pigeon2.getRotation2d();
    }

    @Override public boolean isCurrentlyAlive() {
        return false;
    }


}

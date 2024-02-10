package org.bitbuckets.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro implements IGyro{

    final Pigeon2 pigeon2;

    public Pigeon2Gyro(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;
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

    @Override public void userForceOffset(Rotation2d beAt) {
        tare = tare.plus(userZeroRelativeRotation()).minus(beAt);
    }
}

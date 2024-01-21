package org.bitbuckets.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro implements IGyro{

    final Pigeon2 pigeon2;

    public Pigeon2Gyro(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;
    }

    @Override
    public Rotation2d currentRotation() {
        return pigeon2.getRotation2d();
    }
}

package org.bitbuckets.disabled;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.bitbuckets.drive.Modules;
import org.bitbuckets.drive.IGyro;

public class KinematicGyro implements IGyro {

    final Modules modules;
    final SwerveDriveKinematics kinematics;

    public KinematicGyro(Modules modules, SwerveDriveKinematics kinematics) {
        this.modules = modules;
        this.kinematics = kinematics;

        lastAngle_fieldRelative = Rotation2d.fromDegrees(0);
        lastPositions = modules.currentPositions();
    }

    Rotation2d lastAngle_fieldRelative;
    SwerveModulePosition[] lastPositions;


    @Override
    public Rotation2d rotation_initializationRelative() {

        SwerveModulePosition[] currentPositions = modules.currentPositions();
        SwerveModulePosition[] deltaPositions = delta(currentPositions, lastPositions);

        Rotation2d dTheta = new Rotation2d(kinematics.toTwist2d(deltaPositions).dtheta);

        lastAngle_fieldRelative = lastAngle_fieldRelative.plus(dTheta);
        lastPositions = currentPositions;


        return lastAngle_fieldRelative;
    }

    @Override public boolean isCurrentlyAlive() {
        return true;
    }


    static SwerveModulePosition[] delta(SwerveModulePosition[] now, SwerveModulePosition[] last) {
        SwerveModulePosition[] positions = new SwerveModulePosition[now.length];
        for (int i = 0; i < now.length; i++) {
            positions[i] = new SwerveModulePosition(now[i].distanceMeters - last[i].distanceMeters, now[i].angle);
        }

        return positions;
    }

}

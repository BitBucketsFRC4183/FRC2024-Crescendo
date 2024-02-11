package org.bitbuckets.disabled;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.IGyro;

public class KinematicGyro implements IGyro {

    final DriveSubsystem driveSubsystem;
    final SwerveDriveKinematics kinematics;

    public KinematicGyro(DriveSubsystem driveSubsystem, SwerveDriveKinematics kinematics) {
        this.driveSubsystem = driveSubsystem;
        this.kinematics = kinematics;

        lastAngle_fieldRelative = Rotation2d.fromDegrees(0);
        lastPositions = driveSubsystem.currentPositions();
    }

    Rotation2d lastAngle_fieldRelative;
    SwerveModulePosition[] lastPositions;

    Rotation2d tare = new Rotation2d();

    @Override
    public Rotation2d initializationRelativeRotation() {

        SwerveModulePosition[] currentPositions = driveSubsystem.currentPositions();
        SwerveModulePosition[] deltaPositions = delta(currentPositions, lastPositions);

        Rotation2d dTheta = new Rotation2d(kinematics.toTwist2d(deltaPositions).dtheta);

        lastAngle_fieldRelative = lastAngle_fieldRelative.plus(dTheta);
        lastPositions = currentPositions;


        return lastAngle_fieldRelative;
    }

    @Override
    public Rotation2d userZeroRelativeRotation() {
        return initializationRelativeRotation().minus(tare);
    }

    @Override
    public void userZero() {
        tare = tare.plus(userZeroRelativeRotation());
    }

    @Override public void userForceOffset(Rotation2d beAt) {

    }


    static SwerveModulePosition[] delta(SwerveModulePosition[] now, SwerveModulePosition[] last) {
        SwerveModulePosition[] positions = new SwerveModulePosition[now.length];
        for (int i = 0; i < now.length; i++) {
            positions[i] = new SwerveModulePosition(now[i].distanceMeters - last[i].distanceMeters, now[i].angle);
        }

        return positions;
    }

}

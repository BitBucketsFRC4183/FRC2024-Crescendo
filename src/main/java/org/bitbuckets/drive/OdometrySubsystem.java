package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.Robot;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;

import java.util.Optional;

public class OdometrySubsystem implements Subsystem, IPeriodicLooped {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final SwerveDrivePoseEstimator odometry;
    final IGyro gyro;
    final SwerveDriveKinematics kinematics;

    SwerveModulePosition[] lastPositions;

    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, IGyro gyro, SwerveDriveKinematics kinematics) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.gyro = gyro;

        lastPositions = driveSubsystem.currentPositions();
        mattRegister();
        register();
    }



    @Override
    public void periodic() {
        odometry.update(getGyroAngle(),driveSubsystem.currentPositions());

        if (Robot.isReal()) {
            //VISION
            Optional<Pose3d> visionThinks = visionSubsystem.estimateVisionRobotPose_1();
            if (visionThinks.isPresent()) {
                Pose2d maybeAPose = visionThinks.get().toPose2d();

                odometry.addVisionMeasurement(maybeAPose, MathSharedStore.getTimestamp());
            }
        }


    }


    @Override
    public void logPeriodic() {
        RobotContainer.SWERVE.logPosition(odometry.getEstimatedPosition());
    }

    Rotation2d lastAngle_fieldRelative = Rotation2d.fromDegrees(0);


    static SwerveModulePosition[] delta(SwerveModulePosition[] now, SwerveModulePosition[] last) {
        SwerveModulePosition[] positions = new SwerveModulePosition[now.length];
        for (int i = 0; i < now.length; i++) {
            positions[i] = new SwerveModulePosition(now[i].distanceMeters - last[i].distanceMeters, now[i].angle);
        }

        return positions;
    }
   public Rotation2d getGyroAngle() {
        if (Robot.isSimulation()) {
            SwerveModulePosition[] currentPositions = driveSubsystem.currentPositions();
            SwerveModulePosition[] deltaPositions = delta(currentPositions, lastPositions);

            Rotation2d dTheta = new Rotation2d(kinematics.toTwist2d(deltaPositions).dtheta);

            lastAngle_fieldRelative = lastAngle_fieldRelative.plus(dTheta);
            lastPositions = currentPositions;

            return lastAngle_fieldRelative;
        } else {
            return gyro.currentRotation();
        }
   }

   public Pose2d getCurrentPosition() {
        return odometry.getEstimatedPosition();
   }

   public void forceOdometryToThinkWeAreAt(Pose3d position) {
        odometry.resetPosition(gyro.currentRotation(), driveSubsystem.currentPositions(), position.toPose2d());
   }
}

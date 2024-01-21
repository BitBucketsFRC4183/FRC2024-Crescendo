package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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


    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, IGyro gyro) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.odometry = odometry;
        this.gyro = gyro;

        mattRegister();
        register();
    }



    @Override
    public void periodic() {

        odometry.update(gyro.currentRotation(),driveSubsystem.currentPositions());

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


   public Rotation2d getGyroAngle() {
        if (Robot.isSimulation()) {
            return odometry.getEstimatedPosition().getRotation();
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

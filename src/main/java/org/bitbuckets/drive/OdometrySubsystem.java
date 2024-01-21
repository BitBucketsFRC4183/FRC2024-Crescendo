package org.bitbuckets.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;

import java.util.Optional;

public class OdometrySubsystem implements Subsystem, IPeriodicLooped {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final SwerveDrivePoseEstimator odometry;
    final Pigeon2 pigeon2;



    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, Pigeon2 pigeon2) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.odometry = odometry;
        this.pigeon2 = pigeon2;

        mattRegister();
        register();
    }



    @Override
    public void periodic() {
        odometry.update(pigeon2.getRotation2d(),driveSubsystem.currentPositions());

        //VISION
        Optional<Pose3d> visionThinks = visionSubsystem.estimateVisionRobotPose_1();
        if (visionThinks.isPresent()) {
            Pose2d maybeAPose = visionThinks.get().toPose2d();

            odometry.addVisionMeasurement(maybeAPose, MathSharedStore.getTimestamp());
        }


    }


    @Override
    public void logPeriodic() {
        RobotContainer.SWERVE.logPosition(odometry.getEstimatedPosition());
    }


   public Rotation2d getGyroAngle() {
        return pigeon2.getRotation2d();
   }

   public Pose2d getCurrentPosition() {
        return odometry.getEstimatedPosition();
   }
}

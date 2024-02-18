package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.Robot;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.EstimatedRobotPose;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class OdometrySubsystem implements Subsystem, IMattlibHooked {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final CustomSwervePoseEstimator odometry;
    final SwerveDriveKinematics kinematics;
    final IGyro gyro;

    final Component odometryComponent;

    public interface Component extends INetworkedComponent {
        @Conf("centroid_height") double robotCentroidHeightWrtGround_meters();

        @Log("x_velocity") void reportXVelocity(double xVelocity_metersPerSecond);
        @Log("y_velocity") void reportYVelocity(double yVelocity_metersPerSecond);

        @Conf("fr_pos_offset") Translation2d fr_offset();
        @Conf("fl_pos_offset") Translation2d fl_offset();
        @Conf("br_pos_offset") Translation2d br_offset();
        @Conf("bl_pos_offset") Translation2d bl_offset();
    }

    SwerveModulePosition[] lastPositions_dxdy = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    double lastTimestamp_seconds = 0;

    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CustomSwervePoseEstimator odometry, IGyro gyro, SwerveDriveKinematics kinematics, Component odometryComponent) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.gyro = gyro;
        this.odometryComponent = odometryComponent;


        mattRegister();
        register();
    }

    @Override
    public ExplainedException[] verifyInit() {
        odometry.resetPosition(gyro.initializationRelativeRotation(), driveSubsystem.currentPositions(), new Pose2d());

        return new ExplainedException[0];
    }

    @Override
    public void periodic() {
        odometry.update(gyro.initializationRelativeRotation(),driveSubsystem.currentPositions());

        if (Robot.isReal()) {
            //VISION
            Optional<EstimatedRobotPose> optVisionPose = visionSubsystem.estimateVisionRobotPose();
            if (optVisionPose.isPresent()) {
                Pose2d visionPose = optVisionPose.get().estimatedPose.toPose2d();
                RobotContainer.VISION.log_final_pose(visionPose);
                // odometry.addVisionMeasurement(visionPose, optVisionPose.get().timestampSeconds);
            }
        } else {
            Optional<EstimatedRobotPose> optEsmPose = visionSubsystem.estimateVisionRobotPose();
            optEsmPose.ifPresent(esmPose -> RobotContainer.VISION.log_final_pose(esmPose.estimatedPose.toPose2d()));
        }

        lastPositions_dxdy = driveSubsystem.currentPositions();
        lastTimestamp_seconds = MathSharedStore.getTimestamp();

    }


    @Override
    public void logPeriodic() {
        RobotContainer.SWERVE.logPosition(odometry.getEstimatedPosition());
        odometryComponent.reportXVelocity(robotVelocity_metersPerSecond().vxMetersPerSecond);
        odometryComponent.reportYVelocity(robotVelocity_metersPerSecond().vyMetersPerSecond);
    }


    public Rotation2d getGyroAngle() {
        return gyro.userZeroRelativeRotation();
   }

   public Pose2d getRobotCentroidPosition() {
        return odometry.getEstimatedPosition();
   }

   public Pose3d getRobotCentroidPositionVert() {
        Pose2d estimatedPose = odometry.getEstimatedPosition();

        return new Pose3d(
                estimatedPose.getX(),
                estimatedPose.getY(),
                odometryComponent.robotCentroidHeightWrtGround_meters(),
                new Rotation3d(0,0, estimatedPose.getRotation().getRadians())
        );
   }


   public Pose3d getShooterCentroidPositionVert() {
        //TODO someone needs to do this

       return null;
   }

   public void forceOdometryToThinkWeAreAt(Pose3d position) {
        odometry.resetPosition(gyro.initializationRelativeRotation(), driveSubsystem.currentPositions(), position.toPose2d());
   }

   //i have no idea what this does dont use it
   public void debugZero() {
        gyro.userZero();
   }

   public void debugGyroToPosition(Rotation2d beat) {
        gyro.userForceOffset(beat);
   }

   public ChassisSpeeds robotVelocity_metersPerSecond() {
        SwerveModulePosition[] currentPositions = driveSubsystem.currentPositions();
        SwerveModulePosition[] lastPositions = lastPositions_dxdy;

        double currentTime_seconds = MathSharedStore.getTimestamp();
        double lastTime_seconds = lastTimestamp_seconds;

        double dt = currentTime_seconds - lastTime_seconds;
        Twist2d twist = kinematics.toTwist2d(
                new SwerveDriveWheelPositions(lastPositions),
                new SwerveDriveWheelPositions(currentPositions)
        );

        double dxdt = twist.dx / dt;
        double dydt = twist.dy / dt;
        double dthetadt = twist.dtheta / dt;

        return new ChassisSpeeds(
                dxdt,
                dydt,
                dthetadt
        );
   }
}

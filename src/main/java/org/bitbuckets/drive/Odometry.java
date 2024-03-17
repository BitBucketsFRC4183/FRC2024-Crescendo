package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.bitbuckets.Robot;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.EstimatedRobotPose;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Essential;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class Odometry implements IMattlibHooked {

    final Modules modules;
    final VisionSubsystem visionSubsystem;
    final CustomSwervePoseEstimator odometry;
    final SwerveDriveKinematics kinematics;
    final IGyro gyro;

    final Component odometryComponent;

    public boolean visionOdometry;

    public interface Component extends INetworkedComponent {
        @Conf("pigeon_can_id") int pigeonCanId();

        @Conf("centroid_height") double robotCentroidHeightWrtGround_meters();

        @Log("x_velocity") void reportXVelocity(double xVelocity_metersPerSecond);
        @Log("y_velocity") void reportYVelocity(double yVelocity_metersPerSecond);

        @Conf("fr_pos_offset") Translation2d fr_offset();
        @Conf("fl_pos_offset") Translation2d fl_offset();
        @Conf("br_pos_offset") Translation2d br_offset();
        @Conf("bl_pos_offset") Translation2d bl_offset();

        @Essential @Log("rot_gyro") void logGyroRotation(double rot);
        @Log("rot_odo") void logOdoRotation(double rot);
        @Essential @Log("pose_odo") void logPosition(Pose2d pose2d);
        @Log("pidgeon_ok") void logPidgeonOk(boolean isOk);
    }

    SwerveModulePosition[] lastPositions_dxdy = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    double lastTimestamp_seconds = 0;


    public Odometry(Modules modules, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, IGyro gyro, SwerveDriveKinematics kinematics, Component odometryComponent) {
        this.modules = modules;
        this.visionSubsystem = visionSubsystem;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.gyro = gyro;
        this.odometryComponent = odometryComponent;
        this.visionOdometry = true;

        mattRegister();
    }

    @Override
    public ExplainedException[] verifyInit() {
        odometry.resetPosition(gyro.rotation_initializationRelative(), modules.currentPositions(), new Pose2d());

        return new ExplainedException[0];
    }

    @Override
    public void logicPeriodic() {
        odometry.update(gyro.rotation_initializationRelative(), modules.currentPositions());

        if (Robot.isReal()) {
            //VISION
            Optional<EstimatedRobotPose> optVisionPose = visionSubsystem.estimateVisionRobotPose();
            if (optVisionPose.isPresent()) {
                Pose2d visionPose = optVisionPose.get().estimatedPose.toPose2d();
                RobotContainer.VISION.log_final_pose(visionPose);
                if (this.visionOdometry) {odometry.addVisionMeasurement(visionPose, optVisionPose.get().timestampSeconds);}
            }
        } else {
            Optional<EstimatedRobotPose> optEsmPose = visionSubsystem.estimateVisionRobotPose();
            optEsmPose.ifPresent(esmPose -> RobotContainer.VISION.log_final_pose(esmPose.estimatedPose.toPose2d()));
        }
    }


    @Override
    public void logPeriodic() {
        odometryComponent.logPosition(odometry.getEstimatedPosition());
        odometryComponent.logOdoRotation(odometry.getEstimatedPosition().getRotation().getDegrees());
        odometryComponent.logGyroRotation(gyro.rotation_initializationRelative().getDegrees());
        odometryComponent.logPidgeonOk(gyro.isCurrentlyAlive());
    }


    public Rotation2d getHeading_fieldRelative() {
        return odometry.getEstimatedPosition().getRotation();
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

     public Pose3d getCameraPositionVert() {
        return getRobotCentroidPositionVert().plus(
                new Transform3d(
                        odometryComponent.cameraCentroidOffset(),
                        new Rotation3d()
                )
        );
   }

     public Pose3d getShooterCentroidPositionVert() {

        //TODO someone needs to do this

       return null;
   }

     public void forcePosition(Pose2d position_fieldRelative) {
        odometry.resetPosition(
                gyro.rotation_initializationRelative(),
                modules.currentPositions(),
                position_fieldRelative
        );
     }

    public void forceHeading(Rotation2d rotation_fieldRelative) {

        odometry.resetPosition(
               gyro.rotation_initializationRelative(),
               modules.currentPositions(),
               new Pose2d(
                          odometry.getEstimatedPosition().getTranslation(),
                          rotation_fieldRelative
               )
        );
     }

   public ChassisSpeeds robotVelocity_metersPerSecond() {
        return kinematics.toChassisSpeeds(modules.currentHallEffectStates());

   }
}

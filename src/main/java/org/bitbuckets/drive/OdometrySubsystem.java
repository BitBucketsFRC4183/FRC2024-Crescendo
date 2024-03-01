package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.Robot;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Essential;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class OdometrySubsystem implements Subsystem, IMattlibHooked {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final SwerveDrivePoseEstimator odometry;
    final SwerveDriveKinematics kinematics;
    final IGyro gyro;
    final DigitalInput gyroResetButton;
    public final Trigger gyroResetButtonTrigger;

    final Component odometryComponent;

    public interface Component extends INetworkedComponent {
        @Conf("centroid_height") double robotCentroidHeightWrtGround_meters();
        @Conf("camera_centroid_offset") Translation3d cameraCentroidOffset();

        @Conf("fr_pos_offset") Translation2d fr_offset();
        @Conf("fl_pos_offset") Translation2d fl_offset();
        @Conf("br_pos_offset") Translation2d br_offset();
        @Conf("bl_pos_offset") Translation2d bl_offset();

        @Log("rot_gyro") void logGyroRotation(double rot);
        @Log("rot_odo") void logOdoRotation(double rot);
        @Essential @Log("pose_odo") void logPosition(Pose2d pose2d);
        @Essential @Log("reset_button_state") void logReset(boolean reset);

        @Conf("gyroResetButton_dio") int gyroResetButtonId();
    }



    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, IGyro gyro, SwerveDriveKinematics kinematics, Component odometryComponent) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.gyro = gyro;
        this.odometryComponent = odometryComponent;

        gyroResetButton = new DigitalInput(odometryComponent.gyroResetButtonId());
        gyroResetButtonTrigger = new Trigger(RobotContainer.always,() -> !gyroResetButton.get());

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
            Optional<Pose3d> visionThinks = visionSubsystem.estimateVisionRobotPose_1();
            if (visionThinks.isPresent()) {
                Pose2d maybeAPose = visionThinks.get().toPose2d();

                odometry.addVisionMeasurement(maybeAPose, MathSharedStore.getTimestamp());
            }
        }

    }


    @Override
    public void logPeriodic() {
        odometryComponent.logPosition(odometry.getEstimatedPosition());
        odometryComponent.logOdoRotation(odometry.getEstimatedPosition().getRotation().getRadians());
        odometryComponent.logGyroRotation(odometry.getEstimatedPosition().getRotation().getRadians());
        odometryComponent.logReset(gyroResetButton.get());

    }


    /**
     * Gets the auto-offset independent but user-zero dependent gyro angle. TODO this may break on red
     * @return
     */
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

   public void forceOdometryToThinkWeAreAt(Pose3d position) {
        odometry.resetPosition(
                gyro.initializationRelativeRotation(),
                driveSubsystem.currentPositions(),
                position.toPose2d()
        );
   }

   //i have no idea what this does dont use it
   public void debugZero() {
        gyro.userZero();
   }

   public void debugGyroToPosition(Rotation2d beat) {
        gyro.userForceOffset(beat);
   }

   public Translation2d robotVelocity_metersPerSecond() {
        return null; //TODO
   }
}

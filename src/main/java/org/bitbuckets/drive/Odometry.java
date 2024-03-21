package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.bitbuckets.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.util.FieldConstants;
import org.bitbuckets.vision.VisionSubsystem;
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
    final SwerveDrivePoseEstimator odometry;
    final SwerveDriveKinematics kinematics;
    final IGyro gyro;

    final Component odometryComponent;

    public interface Component extends INetworkedComponent {
        @Conf("pigeon_can_id") int pigeonCanId();

        @Conf("centroid_height") double robotCentroidHeightWrtGround_meters();
        @Conf("camera_centroid_offset") Translation3d cameraCentroidOffset();

        @Conf("fr_pos_offset") Translation2d fr_offset();
        @Conf("fl_pos_offset") Translation2d fl_offset();
        @Conf("br_pos_offset") Translation2d br_offset();
        @Conf("bl_pos_offset") Translation2d bl_offset();

        @Essential @Log("rot_gyro") void logGyroRotation(double rot);
        @Log("rot_odo") void logOdoRotation(double rot);
        @Essential @Log("pose_odo") void logPosition(Pose2d pose2d);
        @Log("pidgeon_ok") void logPidgeonOk(boolean isOk);

        @Log("allianceSpeaker") void logAllianceSpeaker(Pose2d allianceSpeaker);
        @Log("dist_allianceSpeaker") void logDistanceAllianceSpeaker(double dist);

    }



    public Odometry(Modules modules, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry, IGyro gyro, SwerveDriveKinematics kinematics, Component odometryComponent) {
        this.modules = modules;
        this.visionSubsystem = visionSubsystem;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.gyro = gyro;
        this.odometryComponent = odometryComponent;

        mattRegister();
    }

    @Override
    public ExplainedException[] verifyInit() {
        odometry.resetPosition(gyro.rotation_initializationRelative(), modules.currentPositions(), new Pose2d());

        return new ExplainedException[0];
    }

    Pose2d allianceSpeaker = new Pose2d();
    double distance = 0;

    @Override
    public void logicPeriodic() {
        odometry.update(gyro.rotation_initializationRelative(), modules.currentPositions());

        if (Robot.isReal()) {
            //VISION
            Optional<Pose3d> visionThinks = visionSubsystem.estimateVisionRobotPose_1();
            if (visionThinks.isPresent()) {
                Pose2d maybeAPose = visionThinks.get().toPose2d();

                odometry.addVisionMeasurement(maybeAPose, MathSharedStore.getTimestamp());
            }
        }

        Translation2d speakerOpening = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            speakerOpening = new Translation2d(
                    FieldConstants.fieldLength - speakerOpening.getX(),
                    speakerOpening.getY()
            );
        }


        allianceSpeaker = new Pose2d(speakerOpening.getX(), speakerOpening.getY(), new Rotation2d());
        distance =
                getRobotCentroidPosition()
                        .getTranslation()
                        .getDistance(speakerOpening);

    }


    @Override
    public void logPeriodic() {
        odometryComponent.logPosition(odometry.getEstimatedPosition());
        odometryComponent.logOdoRotation(odometry.getEstimatedPosition().getRotation().getDegrees());
        odometryComponent.logGyroRotation(gyro.rotation_initializationRelative().getDegrees());
        odometryComponent.logPidgeonOk(gyro.isCurrentlyAlive());
        odometryComponent.logOdoRotation(odometry.getEstimatedPosition().getRotation().getRadians());
        odometryComponent.logGyroRotation(odometry.getEstimatedPosition().getRotation().getRadians());

        odometryComponent.logAllianceSpeaker(allianceSpeaker);
        odometryComponent.logDistanceAllianceSpeaker(distance);

    }

    public double distanceFromAllianceSpeaker() {
        return distance;
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

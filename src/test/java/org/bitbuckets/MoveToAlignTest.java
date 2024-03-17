package org.bitbuckets;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.bitbuckets.drive.Modules;
import org.bitbuckets.drive.Odometry;
import org.bitbuckets.vision.VisionSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class MoveToAlignTest {
    Modules modules;
    Odometry odometry;
    VisionSubsystem visionSubsystem;
    HolonomicDriveController holonomicDriveController;

    @BeforeEach
    void setup() {
        modules = mock(Modules.class);
        odometry = mock(Odometry.class);
        visionSubsystem = mock(VisionSubsystem.class);
        holonomicDriveController = new HolonomicDriveController(
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(2, 4))
        );
    }

    @Test
    void calculateMoveToAlignForwards() {
        // our pose is 0,0 rotated 0º (facing forwards, away from alliance wall)
        when(odometry.getRobotCentroidPosition()).thenReturn(new Pose2d());

        // get chassis speeds for a target that is at 1, 0
        // we should get chassisSpeeds telling our robot to move forwards, away from the alliance wall
        ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(
                new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(),
                1,
                Rotation2d.fromDegrees(0)
        );

        System.out.println(chassisSpeeds);
        assertEquals(-1, chassisSpeeds.vxMetersPerSecond, .1);
        assertEquals(0, chassisSpeeds.vyMetersPerSecond, .1);
        assertEquals(0, chassisSpeeds.omegaRadiansPerSecond, .1);
    }

    @Test
    void calculateMoveToAlignBackwards() {
        // our pose is 0,0 rotated 180º (facing the alliance wall)
        when(odometry.getRobotCentroidPosition()).thenReturn(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

        // get chassis speeds for a target that is at 1, 0
        // we should get chassisSpeeds telling our robot to move backwards, away from the alliance wall
        ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(
                new Pose2d(1, 0, Rotation2d.fromDegrees(0)), new Pose2d(), 1, Rotation2d.fromDegrees(180));
        assertEquals(-1, chassisSpeeds.vxMetersPerSecond, .1);
        assertEquals(0, chassisSpeeds.vyMetersPerSecond, .1);
        assertEquals(0, chassisSpeeds.omegaRadiansPerSecond, .1);
    }
}

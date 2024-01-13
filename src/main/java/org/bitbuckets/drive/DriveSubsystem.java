package org.bitbuckets.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.Robot;
import org.bitbuckets.util.Util;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;

public class DriveSubsystem implements Subsystem, IPeriodicLooped {

    final SwerveModule[] modules;
    final SwerveDriveKinematics kinematics;
    final SimpleMotorFeedforward ff;
    final VisionSubsystem visionSubsystem;

    public DriveSubsystem(SwerveModule[] modules, SwerveDriveKinematics kinematics, SimpleMotorFeedforward ff, VisionSubsystem visionSubsystem) {
        this.modules = modules;
        this.kinematics = kinematics;
        this.ff = ff;
        this.visionSubsystem = visionSubsystem;

        register();
        mattRegister();
    }


    // periodic looped

    @Override
    public void logPeriodic() {


        Robot.DRIVE.logSwervePositions(currentPositions());
        Robot.DRIVE.logSwerveStates(currentStates());
    }

    /**
     * Commands the motors to drive at some voltages, using a chassis speed reference
     * This will set them for the rest of time
     */
    public void driveUsingChassisSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        driveUsingSwerveStates(states);
    }

    /**
     * Commands each module in the module array to move using the swerve module states as reference
     * @param states states indexed by the IDs at the top of this class
     */
    public void driveUsingSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            double feedforwardVoltage = ff.calculate(states[i].speedMetersPerSecond);
            feedforwardVoltage = MathUtil.clamp(feedforwardVoltage, -Util.MAX_VOLTAGE, Util.MAX_VOLTAGE);
            double referenceAngle = states[i].angle.getRotations();

            modules[i].setToMoveAt(feedforwardVoltage, referenceAngle);
        }
    }


    /**
     *
     * @return The current position of the swerve drive, as reported by each module
     */
    public SwerveModulePosition[] currentPositions() {
        return new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()
        };
    }


    /**
     *
     * @return The current state of the swerve drive as reported by each module
     */
    public SwerveModuleState[] currentStates() {
        return new SwerveModuleState[] {
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()
        };
    }

    public void commandWheelsToZero() {
        for (SwerveModule module : modules) {
            module.stopAllMotors();
        }
    }

    public void moveToAlign() {
        var tagPose_1 = visionSubsystem.estimateBestVisionTarget_1();
        if (tagPose_1.isPresent()) {
            //do drive things
        }
    }

}

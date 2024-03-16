package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.log.annote.Tune;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class SwerveSubsystem implements Subsystem, IMattlibHooked {

    // HARDWARE
    public final Modules modules;
    public final Odometry odometry;
    final Component swerveComponent;

    public interface Component extends INetworkedComponent {
        @Tune("use_velocity_pid_teleop") boolean useVelocityPidTeleop();
        @Tune("use_offset_finding_mode") boolean useOffsetFindingMode(); //turn on when trying to figure out which way is "forward" for modules when no offsets are present. This is nice because if you don't have it SwerveModuleState.optimize's flipping will make it impossible for you to tell where forward really is
        @Tune("p_heading") double p_heading();

        @Log("isAtGoal") void reportAtGoal(boolean atGoal);
    }

    // STATE
    double lastPeriodicTime_seconds = MathSharedStore.getTimestamp();
    double lastTimeWithThetaMovement_seconds = MathSharedStore.getTimestamp();

    boolean autonomousLikeMode = false;
    ChassisSpeeds userSetpoint_fieldRelative = new ChassisSpeeds();
    Rotation2d userHeading_fieldRelative = Rotation2d.fromDegrees(0);



    public SwerveSubsystem(Modules modules, Odometry odometry, Component component) {
        this.modules = modules;
        this.odometry = odometry;
        this.swerveComponent = component;

        register();
        mattRegister();
    }

    @Override
    public void logicPeriodic() {
        //handle time
        double now_seconds = MathSharedStore.getTimestamp();
        ChassisSpeeds mutableSetpoint_fieldRelative = userSetpoint_fieldRelative;

        if (!autonomousLikeMode) {
            mutableSetpoint_fieldRelative = headingCorrectSpeeds(mutableSetpoint_fieldRelative, now_seconds);
        }

        SwerveModuleState[] setpointStates = odometry.kinematics.toSwerveModuleStates(mutableSetpoint_fieldRelative);
        Rotation2d[] headings = modules.currentModuleHeadings();

        if (!swerveComponent.useOffsetFindingMode()) {
            for (int i = 0; i < setpointStates.length; i++) {
                setpointStates[i] = SwerveModuleState.optimize(setpointStates[i], headings[i]);
            }
        }


        //DRIVING CODE
        modules.driveUsingSwerveStates(setpointStates, swerveComponent.useVelocityPidTeleop());


        //update old status
        lastPeriodicTime_seconds = now_seconds;
    }


    /**
     * Used for auto
     * @param speeds_fieldRelative
     */
    public void orderToUnfiltered(ChassisSpeeds speeds_fieldRelative) {
        userSetpoint_fieldRelative = speeds_fieldRelative;
        userHeading_fieldRelative = odometry.getHeading_fieldRelative();
        autonomousLikeMode = true;
    }

    public void orderTo(ChassisSpeeds speeds_fieldRelative, Rotation2d desiredHeading_fieldRelative) {
        userSetpoint_fieldRelative = speeds_fieldRelative;
        userHeading_fieldRelative = desiredHeading_fieldRelative;
        autonomousLikeMode = false;
    }


    public void orderTo(ChassisSpeeds speeds_fieldReference) {
        userSetpoint_fieldRelative = speeds_fieldReference;
        userHeading_fieldRelative = odometry.getHeading_fieldRelative();
        autonomousLikeMode = false;
    }

    public void orderToZero() {
        userSetpoint_fieldRelative = new ChassisSpeeds();
        userHeading_fieldRelative = odometry.getHeading_fieldRelative();
        autonomousLikeMode = false;
    }



    /**
     * shit makes pid go straight
     * side effects: causes user heading to go idk where LOL
     * derivatives and shit
     * @param speeds
     * @param now_seconds
     * @return
     */
    ChassisSpeeds headingCorrectSpeeds(ChassisSpeeds speeds, double now_seconds) {
        double dt_seconds = now_seconds - lastPeriodicTime_seconds;
        double dTheta_radianSeconds = userSetpoint_fieldRelative.omegaRadiansPerSecond;

        // HEADING CORRECTION
        if (Math.abs(userSetpoint_fieldRelative.omegaRadiansPerSecond) > 0.01) {
            //if we are turning do not use heading correction
            lastTimeWithThetaMovement_seconds = now_seconds;
            return speeds;
        }

        if (now_seconds - lastTimeWithThetaMovement_seconds < 0.5) {
            //wait until 0.5 seconds have passed of no user-commanded movement before initializing the heading correction
            return speeds;
        }

        userHeading_fieldRelative = userHeading_fieldRelative.plus(new Rotation2d(dTheta_radianSeconds * dt_seconds));
        Rotation2d errorHeading = userHeading_fieldRelative.minus(odometry.getHeading_fieldRelative());

        if (Math.abs(errorHeading.getDegrees()) < 2) {
            swerveComponent.reportAtGoal(true);
            return speeds;
        } else {
            swerveComponent.reportAtGoal(false);
        }

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, errorHeading.getRadians() / dt_seconds * swerveComponent.p_heading());
    }


}

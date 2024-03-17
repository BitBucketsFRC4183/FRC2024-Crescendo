package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Tune;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class DriveSubsystem implements Subsystem, IMattlibHooked {

    // HARDWARE
    public final Modules modules;
    public final Odometry odometry;
    public final Component swerveComponent;

    public interface Component extends INetworkedComponent {
        @Tune("use_velocity_pid_teleop") boolean useVelocityPidTeleop();
        @Tune("use_offset_finding_mode") boolean useOffsetFindingMode(); //turn on when trying to figure out which way is "forward" for modules when no offsets are present. This is nice because if you don't have it SwerveModuleState.optimize's flipping will make it impossible for you to tell where forward really is
    }

    public DriveSubsystem(Modules modules, Odometry odometry, Component component) {
        this.modules = modules;
        this.odometry = odometry;
        this.swerveComponent = component;

        register();
        mattRegister();
    }



    /**
     * Used for auto
     * @param speeds_fieldRelative
     */
    public void orderToUnfilteredAuto(ChassisSpeeds speeds_fieldRelative) {
        SwerveModuleState[] setpointStates = odometry.kinematics.toSwerveModuleStates(speeds_fieldRelative);
        Rotation2d[] headings = modules.currentModuleHeadings();

        if (!swerveComponent.useOffsetFindingMode()) {
            for (int i = 0; i < setpointStates.length; i++) {
                setpointStates[i] = SwerveModuleState.optimize(setpointStates[i], headings[i]);
            }
        }


        modules.driveUsingSwerveStates(setpointStates, true);
    }


    /**
     * Used for auto
     * @param speeds_fieldRelative
     */
    public void orderToUnfiltered(ChassisSpeeds speeds_fieldRelative) {
        SwerveModuleState[] setpointStates = odometry.kinematics.toSwerveModuleStates(speeds_fieldRelative);
        Rotation2d[] headings = modules.currentModuleHeadings();

        if (!swerveComponent.useOffsetFindingMode()) {
            for (int i = 0; i < setpointStates.length; i++) {
                setpointStates[i] = SwerveModuleState.optimize(setpointStates[i], headings[i]);
            }
        }


        modules.driveUsingSwerveStates(setpointStates, swerveComponent.useVelocityPidTeleop());
    }

    public void orderToHeadingOnly(ChassisSpeeds speeds_onlyHeading) {
        SwerveModuleState[] setpointStates = odometry.kinematics.toSwerveModuleStates(speeds_onlyHeading);

        modules.driveUsingHeading(setpointStates);
    }

    public void orderToZero() {
        orderToUnfilteredAuto(new ChassisSpeeds(0,0,0));
    }

    public Command orderToZeroCommand() {
        return Commands.runOnce(this::orderToZero, this);
    }


}

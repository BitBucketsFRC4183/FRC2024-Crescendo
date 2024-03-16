package org.bitbuckets.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class DriveSubsystem implements Subsystem, IMattlibHooked {

    public final SwerveModule[] modules;
    final SwerveDriveKinematics kinematics;
    //final UngodlyAbomination swerveGenerator;


    public DriveSubsystem(SwerveModule[] modules, SwerveDriveKinematics kinematics) {
        this.modules = modules;
        this.kinematics = kinematics;
        //this.swerveGenerator = swerveGenerator;

        register();
        mattRegister();
    }

    @Override
    public void logPeriodic() {
        if (desiredStates != null) {
            RobotContainer.SWERVE.logDesiredStates(desiredStates);
        }
        RobotContainer.SWERVE.logAbsoluteBasedStates(currentAbsoluteBasedState());
        RobotContainer.SWERVE.logHallEncoderBasedStates(currentHallEffectStates());

    }

    @Override public void logicPeriodic() {
/*
        lastSetpoint = swerveGenerator.generateSetpoint(
                LIM,
                lastSetpoint,
                toFollow,
                0.02
        );

        SwerveModuleState[] states = lastSetpoint.moduleStates();
        driveUsingSwerveStates(states, false);*/
    }
/*
    UngodlyAbomination.SwerveSetpoint lastSetpoint = new UngodlyAbomination.SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[]{
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            }
    );


    public static final UngodlyAbomination.ModuleLimits LIM =  new UngodlyAbomination.ModuleLimits(
            4.4,
            4.4 * 5,
            Units.degreesToRadians(1080.0)
    );*/

    ChassisSpeeds toFollow = new ChassisSpeeds();

    /**
     * Commands the motors to drive at some voltages, using a chassis speed reference
     * This will set them for the rest of time
     */
    public void driveUsingChassisSpeed(ChassisSpeeds speeds_robotRelative, boolean usePID) {
        toFollow = speeds_robotRelative;

        driveUsingSwerveStates(kinematics.toSwerveModuleStates(speeds_robotRelative), usePID);

    }

    /**
     * Commands each module in the module array to move using the swerve module states_robotRelative as reference
     * @param states_robotRelative states_robotRelative indexed by the IDs at the top of this class
     */
    public void driveUsingSwerveStates(SwerveModuleState[] states_robotRelative, boolean usePID) {
        desiredStates = states_robotRelative;


        for (int i = 0; i < modules.length; i++) {
            modules[i].setToMoveAt(states_robotRelative[i], usePID);
        }
    }

    SwerveModuleState[] desiredStates;


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
    public SwerveModuleState[] currentHallEffectStates() {
        return new SwerveModuleState[] {
                modules[0].getHallEffectBasedState(),
                modules[1].getHallEffectBasedState(),
                modules[2].getHallEffectBasedState(),
                modules[3].getHallEffectBasedState()
        };
    }

    public SwerveModuleState[] currentAbsoluteBasedState() {
        return new SwerveModuleState[] {
                modules[0].getAbsoluteBasedState(),
                modules[1].getAbsoluteBasedState(),
                modules[2].getAbsoluteBasedState(),
                modules[3].getAbsoluteBasedState()
        };
    }

    public void commandWheelsToZero() {
        for (SwerveModule module : modules) {
            module.stopAllMotors();
        }
    }



}

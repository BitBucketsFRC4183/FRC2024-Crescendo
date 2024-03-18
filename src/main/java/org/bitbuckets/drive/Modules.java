package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.bitbuckets.RobotContainer;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class Modules implements IMattlibHooked {

    public final SwerveModule[] modules;
    final Component component;

    public interface Component extends INetworkedComponent {
        @Log("desired_states") void logDesiredStates(SwerveModuleState[] desiredStates);
        @Log("hall_states") void logHallEncoderBasedStates(SwerveModuleState[] states);
        @Log("absolute_states") void logAbsoluteBasedStates(SwerveModuleState[] states);
    }

    public Modules(SwerveModule[] modules, Component component) {
        this.modules = modules;
        this.component = component;

        mattRegister();
    }

    SwerveModuleState[] desiredStates_forLog;

    @Override
    public void logPeriodic() {
        if (desiredStates_forLog != null) {
            component.logDesiredStates(desiredStates_forLog);
        }
        component.logAbsoluteBasedStates(currentAbsoluteBasedState());
        component.logHallEncoderBasedStates(currentHallEffectStates());
    }

    /**
     * Commands each module in the module array to move using the swerve module states_robotRelative as reference
     * @param states_robotRelative states_robotRelative indexed by the IDs at the top of this class
     */
    public void driveUsingSwerveStates(SwerveModuleState[] states_robotRelative, boolean usePID) {
        desiredStates_forLog = states_robotRelative;

        for (int i = 0; i < modules.length; i++) {
            modules[i].setToMoveAt(states_robotRelative[i], usePID);
        }
    }

    public void driveUsingFutureStates(SwerveModuleState[] statesNow, SwerveModuleState[] statesLater) {
        desiredStates_forLog = statesNow;

        for (int i = 0; i < modules.length; i++) {
            modules[i].setToMoveAtFuture(statesNow[i], statesLater[i]);
        }
    }

    public void driveUsingHeading(SwerveModuleState[] states) {
        desiredStates_forLog = states;

        for (int i = 0; i < modules.length; i++) {
            modules[i].setToHeading(states[i]);
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

    public Rotation2d[] currentModuleHeadings() {
        return new Rotation2d[] {
                modules[0].getAbsoluteBasedState().angle,
                modules[1].getAbsoluteBasedState().angle,
                modules[2].getAbsoluteBasedState().angle,
                modules[3].getAbsoluteBasedState().angle
        };
    }
    public void commandWheelsToZero() {
        for (SwerveModule module : modules) {
            module.stopAllMotors();
        }
    }



}

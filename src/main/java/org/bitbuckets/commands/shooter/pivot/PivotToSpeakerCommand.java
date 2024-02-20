package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.PivotSubsystem;

public class PivotToSpeakerCommand extends Command {
        private final PivotSubsystem flywheelSubsystem;

        public PivotToSpeakerCommand(PivotSubsystem flywheelSubsystem) {
            this.flywheelSubsystem = flywheelSubsystem;
        }

        @Override
        public void initialize() {

        }

        // this angle needs to be tuned for speaker (60 is only a placeholder)
        @Override
        public void execute() {

            double mechanism_rotations = 1/6;
            flywheelSubsystem.moveToRotation(mechanism_rotations);
        }

        @Override
        public boolean isFinished() {
            return flywheelSubsystem.hasReachedAngle(1/6);
        }

        @Override
        public void end(boolean interrupted) {
            flywheelSubsystem.moveToRotation(0);
        }
    }



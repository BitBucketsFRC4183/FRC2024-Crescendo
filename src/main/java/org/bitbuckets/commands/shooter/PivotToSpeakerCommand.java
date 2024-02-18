package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class PivotToSpeakerCommand extends Command {
        private final ShooterSubsystem shooterSubsystem;

        public PivotToSpeakerCommand(ShooterSubsystem shooterSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
        }

        @Override
        public void initialize() {

        }

        // this angle needs to be tuned for speaker (60 is only a placeholder)
        @Override
        public void execute() {

            double mechanism_rotations = 1/6;
            shooterSubsystem.moveToRotation(mechanism_rotations);
        }

        @Override
        public boolean isFinished() {
            return shooterSubsystem.hasReachedAngle(1/6);
        }

        @Override
        public void end(boolean interrupted) {
            shooterSubsystem.moveToRotation(0);
        }
    }



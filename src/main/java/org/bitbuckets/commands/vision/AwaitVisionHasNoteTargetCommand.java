package org.bitbuckets.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.util.RunningBooleanBuffer;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.loop.ISubroutine;

public class AwaitVisionHasNoteTargetCommand extends Command {

    final VisionSubsystem visionSubsystem;
    final RunningBooleanBuffer runningBooleanBuffer = new RunningBooleanBuffer(6);


    public AwaitVisionHasNoteTargetCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        runningBooleanBuffer.clear();
    }

    @Override
    public void execute() {
        runningBooleanBuffer.flag(visionSubsystem.getClosestNotePose().isPresent());
    }

    @Override
    public boolean isFinished() {
        return runningBooleanBuffer.averageBooleans(4);
    }
}

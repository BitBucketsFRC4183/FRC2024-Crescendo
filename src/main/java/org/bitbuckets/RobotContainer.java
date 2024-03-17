package org.bitbuckets;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.climber.ClimberComponent;
import org.bitbuckets.climber.ClimberSubsystem;
import org.bitbuckets.commands.CommandComponent;
import org.bitbuckets.commands.ReadyWhileMovingGroundIntakeCommand;
import org.bitbuckets.commands.ReadyWhileMovingShootCommand;
import org.bitbuckets.commands.climber.MoveClimberCommand;
import org.bitbuckets.commands.drive.BaseDriveCommand;
import org.bitbuckets.commands.drive.DriveFacingStaticPosCommand;
import org.bitbuckets.commands.drive.SitFacingCommand;
import org.bitbuckets.commands.drive.odo.PlaceAllianceZeroHeading;
import org.bitbuckets.commands.drive.odo.PlaceOdometryCommand;
import org.bitbuckets.commands.drive.traj.FollowTrajectoryExactCommand;
import org.bitbuckets.commands.groundIntake.BasicGroundIntakeCommand;
import org.bitbuckets.commands.groundIntake.FeedGroundIntakeGroup;
import org.bitbuckets.commands.groundIntake.GroundOuttakeCommand;
import org.bitbuckets.commands.shooter.AmpMakeReadyGroup;
import org.bitbuckets.commands.shooter.FireMakeReadyGroup;
import org.bitbuckets.commands.shooter.SourceConsumerGroup;
import org.bitbuckets.commands.shooter.flywheel.SpinFlywheelIndefinite;
import org.bitbuckets.disabled.DisablerComponent;
import org.bitbuckets.disabled.KinematicGyro;
import org.bitbuckets.drive.*;
import org.bitbuckets.groundIntake.GroundIntakeComponent;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementComponent;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.util.*;
import org.bitbuckets.vision.CamerasComponent;
import org.bitbuckets.vision.VisionComponent;
import org.bitbuckets.vision.VisionSimContainer;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xyz.auriium.mattlib.ctre.HardwareCTRE;
import xyz.auriium.mattlib2.MattConsole;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibSettings;
import xyz.auriium.mattlib2.auto.ff.RotationFFGenRoutine;
import xyz.auriium.mattlib2.auto.ff.config.GenerateFFComponent;
import xyz.auriium.mattlib2.auto.pid.LinearPIDBrain;
import xyz.auriium.mattlib2.auto.pid.RotationalPIDBrain;
import xyz.auriium.mattlib2.hardware.*;
import xyz.auriium.mattlib2.hardware.config.*;
import xyz.auriium.mattlib2.log.ConsoleComponent;
import xyz.auriium.mattlib2.loop.CTowerCommands;
import xyz.auriium.mattlib2.rev.HardwareREV;
import xyz.auriium.mattlib2.sim.HardwareSIM;
import xyz.auriium.mattlib2.utils.MockingUtil;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.io.IOException;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class RobotContainer {


    public static final EventLoop always = new EventLoop();
    public final Modules modules;
    public final OperatorInput operatorInput;
    public final Translation2d[] translation2ds;
    public final FlywheelSubsystem flywheelSubsystem;
    public final Odometry odometry;
    public final DriveSubsystem swerveSubsystem;
    public final OperatorToSpeeds o2s;
    //m

    public final VisionSubsystem visionSubsystem;
    public final VisionSimContainer visionSimContainer;
    public final ClimberSubsystem climberSubsystem;
    public final GroundIntakeSubsystem groundIntakeSubsystem;
    public final NoteManagementSubsystem noteManagementSubsystem;
    public final SwerveDriveKinematics kinematics;

    public final SendableChooser<Command> chooser;

    public final LinearPIDBrain xController;
    public final LinearPIDBrain yController;
    public final RotationalPIDBrain thetaController;

    public final MattConsole mainConsole;

    Command cachedCurrentlyRunningAutoCommand = null;

    public RobotContainer() {

        RobotController.setBrownoutVoltage(6);

        //DO SETTINGS BEFORE PRE INIT
        MattlibSettings.USE_TELEMETRY = MattlibSettings.LogLevel.VERBOSE_TELEMETRY;
        if (DriverStation.isFMSAttached()) {
            MattlibSettings.USE_TELEMETRY = MattlibSettings.LogLevel.ESSENTIAL_TELEMETRY; //never change this
        };
        MattlibSettings.ROBOT = WhichRobotUtil.loadRobot();

        //THIS HAS TO RUN FIRST
        Mattlib.LOOPER.runPreInit();
        CommandScheduler.getInstance().enable();

        this.xController = new LinearPIDBrain(DRIVE_X_PID);
        this.yController = new LinearPIDBrain(DRIVE_Y_PID);
        this.thetaController = new RotationalPIDBrain(DRIVE_T_PID);

        // load order matters!!!!!
        this.operatorInput = new OperatorInput();
        this.translation2ds = loadTranslations();
        this.kinematics = loadKinematics();
        this.modules = loadModules();
        this.visionSubsystem = loadVisionSubsystem();
        this.odometry = loadOdometrySubsystem();
        this.o2s = new OperatorToSpeeds(operatorInput, O2S);
        this.swerveSubsystem = new DriveSubsystem(modules, odometry, SWERVE);
        this.flywheelSubsystem = loadFlywheelSubsystem();
        //this.pivotSubsystem = loadPivotSubsystem();
        this.climberSubsystem = loadClimberSubsystem();
        this.groundIntakeSubsystem = loadGroundIntakeSubsystem();
        this.noteManagementSubsystem = loadNoteManagementSubsystem();

        if (!DISABLER.vision_disabled() && Robot.isSimulation()) {
            PhotonCamera[] cameras = visionSubsystem.getCameras();
            this.visionSimContainer = new VisionSimContainer(
                    visionSubsystem,
                    odometry,
                    cameras[0],
                    cameras[1],
                    visionSubsystem.layout
            );
        } else this.visionSimContainer = null;

        loadCommands();
        chooser = loadAutonomous();
        mainConsole = new MattConsole(CONSOLE);

        //THIS HAS TO RUN AT THE END
        Mattlib.LOOPER.runPostInit();
        var ee = Mattlib.LOOPER.runPostInit();

        for (ExplainedException e : ee) {
            throw e;
        }

        mainConsole.reportExceptions(ee);


        // disable the annoying driverstation joystick warning
        DriverStation.silenceJoystickConnectionWarning(true);
    }


    public void robotPeriodic() {
        always.poll();
    }

    public void simulationPeriodic() {
        if (DISABLER.vision_disabled()) return;
        this.visionSimContainer.simulationPeriodic();
    }


    public void autonomousInit() {
        operatorInput.actuallyIsTeleop = false;

        cachedCurrentlyRunningAutoCommand = chooser.getSelected();
        cachedCurrentlyRunningAutoCommand.schedule();
    }

    public void disabledInit() {
        if (cachedCurrentlyRunningAutoCommand != null && cachedCurrentlyRunningAutoCommand.isScheduled()) {
            cachedCurrentlyRunningAutoCommand.cancel();
        }

        operatorInput.actuallyIsTeleop = false;
    }

    public void teleopInit() {
        if (cachedCurrentlyRunningAutoCommand != null) {
            cachedCurrentlyRunningAutoCommand.cancel();
        }

        operatorInput.actuallyIsTeleop = true;
    }

    public void testInit() {

        if (cachedCurrentlyRunningAutoCommand != null && cachedCurrentlyRunningAutoCommand.isScheduled()) {
            cachedCurrentlyRunningAutoCommand.cancel();
        }




//        new AwaitThetaCommand(driveSubsystem, odometrySubsystem, thetaController, DRIVE_T_PID, Rotation2d.fromDegrees(90).getRadians())
//                .andThen(Commands.runOnce(() -> {System.out.println("WE ARE DONE");})).schedule();

        //LinearFFGenRoutine groundTopFFRoutine = new LinearFFGenRoutine(TOP_GROUND_FFGEN, groundIntakeSubsystem.topMotor, groundIntakeSubsystem.topMotor);
        //LinearFFGenRoutine groundBottomFFRoutine = new LinearFFGenRoutine(BOTTOM_GROUND_FFGEN, groundIntakeSubsystem.bottomMotor, groundIntakeSubsystem.bottomMotor);
        //CTowerCommands.wrapRoutine(groundTopFFRoutine).schedule();
        //CTowerCommands.wrapRoutine(groundBottomFFRoutine).schedule();
        RotationFFGenRoutine leftShooterR = new RotationFFGenRoutine(LEFT_FLYWHEEL_GEN, flywheelSubsystem.leftMotor, flywheelSubsystem.velocityEncoderLeft);
        RotationFFGenRoutine rightShooterR = new RotationFFGenRoutine(RIGHT_FLYWHEEL_GEN, flywheelSubsystem.rightMotor, flywheelSubsystem.velocityEncoderRight);

        new ParallelCommandGroup(
                CTowerCommands.wrapRoutine(leftShooterR),
                CTowerCommands.wrapRoutine(rightShooterR)
        ).schedule();

//        Command[] commands = new Command[4];
//        for (int i = 0; i < 4; i++) {
//            var motorWeAreTesting = driveSubsystem.modules[i].driveMotor;
//            LinearFFGenRoutine driveFFRoutine = new LinearFFGenRoutine(DRIVE_MOTORS_FFGEN[i],motorWeAreTesting, motorWeAreTesting);
//            commands[i]= CTowerCommands.wrapRoutine(driveFFRoutine);
//        }
//
//        new ParallelCommandGroup(commands).schedule();
    }



    public Command followTrajectory(ChoreoTrajectory trajectory) {
        return new FollowTrajectoryExactCommand(
                trajectory,
                swerveSubsystem,
                xController,
                yController,
                thetaController
        ).andThen(Commands.runOnce(modules::commandWheelsToZero));
    }

    public Command twoNoteStyle(String real, double ramFireSpeed, double deadline_seconds) {

        ChoreoTrajectory[] twoNoteArr = TrajLoadingUtil.getAllTrajectories(real);

        return new SequentialCommandGroup(
                new PlaceOdometryCommand(twoNoteArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(twoNoteArr[0]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(twoNoteArr[1]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                )
        );
    }

    public Command twoNoteCompatStyle(String real, double ramFireSpeed, double deadline_seconds) {

        ChoreoTrajectory[] twoNoteArr = TrajLoadingUtil.getAllTrajectories(real);

        return new SequentialCommandGroup(
                new PlaceOdometryCommand(twoNoteArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                followTrajectory(twoNoteArr[0]),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(twoNoteArr[1]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(twoNoteArr[2]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                )
        );
    }



    SendableChooser<Command> loadAutonomous() {
        double ramFireSpeed = COMMANDS.ramFireSpeed_mechanismRotationsPerSecond();
        double deadline_seconds = COMMANDS.groupDeadline_seconds();



        var twoNoteCompatLeft = twoNoteCompatStyle("twoNoteCompatLeft", ramFireSpeed, deadline_seconds);
        var twoNoteCompatRight = twoNoteCompatStyle("twoNoteCompatRight", ramFireSpeed, deadline_seconds);


        ChoreoTrajectory[] shootLeaveArr = TrajLoadingUtil.getAllTrajectories("shootGetFar");

        var shootLeave = new SequentialCommandGroup(
                new PlaceOdometryCommand(shootLeaveArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                followTrajectory(shootLeaveArr[0])
        );



        ChoreoTrajectory[] threeNoteArr = TrajLoadingUtil.getAllTrajectories("threeNote");


        var threeNote = new SequentialCommandGroup(
                new PlaceOdometryCommand(threeNoteArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(threeNoteArr[0]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(threeNoteArr[1]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(threeNoteArr[2]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(threeNoteArr[3]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                )
        );


        //var fourNoteAugment = twoNoteCompatStyle("fourNoteAugment",ramFireSpeed, deadline_seconds);


        ChoreoTrajectory[] fourNoteArr = TrajLoadingUtil.getAllTrajectories("fourNote");

        var fourNote = new SequentialCommandGroup(
                new PlaceOdometryCommand(fourNoteArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(fourNoteArr[0]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(fourNoteArr[1]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(fourNoteArr[2]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(fourNoteArr[3]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                followTrajectory(fourNoteArr[4]),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(fourNoteArr[5]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(fourNoteArr[6]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                )
        );
        ChoreoTrajectory[] fourNoteCompatLeftArr = TrajLoadingUtil.getAllTrajectories("fourNoteCompatLeft");

        var fourNoteCompatLeft = new SequentialCommandGroup(
                    new PlaceOdometryCommand(fourNoteCompatLeftArr[0], odometry),
                    new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                    followTrajectory(fourNoteCompatLeftArr[0]),
                    new ReadyWhileMovingGroundIntakeCommand(
                            followTrajectory(fourNoteCompatLeftArr[1]),
                            noteManagementSubsystem, groundIntakeSubsystem
                    ),
                    new ReadyWhileMovingShootCommand(
                            followTrajectory(fourNoteCompatLeftArr[2]),
                            flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                    ),
                    new ReadyWhileMovingGroundIntakeCommand(
                            followTrajectory(fourNoteCompatLeftArr[3]),
                            noteManagementSubsystem, groundIntakeSubsystem
                    ),
                    new ReadyWhileMovingShootCommand(
                            followTrajectory(fourNoteCompatLeftArr[4]),
                            flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                    ),
                    new ReadyWhileMovingGroundIntakeCommand(
                            followTrajectory(fourNoteCompatLeftArr[5]),
                            noteManagementSubsystem, groundIntakeSubsystem
                    )
//                    new ReadyWhileMovingShootCommand(
//                            followTrajectory(fourNoteCompatLeftArr[6]),
//                            flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
//                    )

        );

        ChoreoTrajectory[] fourNoteCompatRightArr = TrajLoadingUtil.getAllTrajectories("fourNoteCompatRight");

//        var fourNoteCompatRight = new SequentialCommandGroup(
//                new PlaceOdometryCommand(fourNoteCompatRightArr[0], odometrySubsystem),
//                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
//                followTrajectory(fourNoteCompatRightArr[0]),
//                new ReadyWhileMovingGroundIntakeCommand(
//                        followTrajectory(fourNoteCompatRightArr[1]),
//                        noteManagementSubsystem, groundIntakeSubsystem
//                ),
//                new ReadyWhileMovingShootCommand(
//                        followTrajectory(fourNoteCompatRightArr[2]),
//                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
//                ),
//                followTrajectory(fourNoteCompatRightArr[3]),
//                followTrajectory(fourNoteCompatRightArr[4]),
//                new ReadyWhileMovingGroundIntakeCommand(
//                        followTrajectory(fourNoteCompatRightArr[5]),
//                        noteManagementSubsystem, groundIntakeSubsystem
//                ),
//                followTrajectory(fourNoteCompatRightArr[6]),
//                new ReadyWhileMovingShootCommand(
//                        followTrajectory(fourNoteCompatRightArr[7]),
//                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
//                ),
//                followTrajectory(fourNoteCompatRightArr[8]),
//                followTrajectory(fourNoteCompatRightArr[9]),
//                followTrajectory(fourNoteCompatRightArr[10]),
//                new ReadyWhileMovingGroundIntakeCommand(
//                        followTrajectory(fourNoteCompatRightArr[11]),
//                        noteManagementSubsystem, groundIntakeSubsystem
//                ),
//                followTrajectory(fourNoteCompatRightArr[12]),
//                followTrajectory(fourNoteCompatRightArr[13]),
//                new ReadyWhileMovingShootCommand(
//                        followTrajectory(fourNoteCompatRightArr[14]),
//                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
//                )
//
//
//        );

        ChoreoTrajectory[] sixNoteArr = TrajLoadingUtil.getAllTrajectories("sixNote");

        var sixNote = new SequentialCommandGroup(
                new PlaceOdometryCommand(sixNoteArr[0], odometry),
                new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed),
                followTrajectory(sixNoteArr[0]),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(sixNoteArr[1]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(sixNoteArr[2]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(sixNoteArr[3]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(sixNoteArr[4]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(sixNoteArr[5]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(sixNoteArr[6]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                ),
                new ReadyWhileMovingGroundIntakeCommand(
                        followTrajectory(sixNoteArr[7]),
                        noteManagementSubsystem, groundIntakeSubsystem
                ),
                new ReadyWhileMovingShootCommand(
                        followTrajectory(sixNoteArr[8]),
                        flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, ramFireSpeed, deadline_seconds
                )
        );


        SendableChooser<Command> chooser = new SendableChooser<>();
        //chooser.addOption("twoNote", twoNote);
        chooser.addOption("shootGetFar", shootLeave);
        chooser.addOption("threeNote", threeNote);
        chooser.setDefaultOption("fourNote", fourNote);
        chooser.addOption("doNothing", Commands.waitSeconds(1));
        chooser.addOption("twoNoteCompatRight", twoNoteCompatRight);
        chooser.addOption("twoNoteCompatLeft", twoNoteCompatLeft);
        chooser.addOption("fourNoteCompatLeft", fourNoteCompatLeft);
//        chooser.addOption("fourNoteCompatRight", fourNoteCompatRight);
        chooser.addOption("sixNote", sixNote);

        SmartDashboard.putData("Path", chooser);
        return chooser;
    }

    void loadCommands() {


        //DRIVER STUFF
        operatorInput.movementNotDesired.and(operatorInput.customHeadingNotDesired).onTrue(swerveSubsystem.orderToZeroCommand());
        operatorInput.movementNotDesired.and(operatorInput.ampHeadingHold).whileTrue(new SitFacingCommand(thetaController, swerveSubsystem, Rotation2d.fromDegrees(90), false));
        operatorInput.movementNotDesired.and(operatorInput.rightSpeakerHeadingHold).whileTrue(new SitFacingCommand(thetaController, swerveSubsystem, Rotation2d.fromRadians(Math.PI / 3), true));
        operatorInput.movementNotDesired.and(operatorInput.leftSpeakerHeadingHold).whileTrue(new SitFacingCommand(thetaController, swerveSubsystem, Rotation2d.fromRadians(-Math.PI / 3), true));
        operatorInput.movementNotDesired.and(operatorInput.frontSpeakerHeadingHold).whileTrue(new SitFacingCommand(thetaController, swerveSubsystem, Rotation2d.fromDegrees(0), true));

        operatorInput.movementDesired.and(operatorInput.customHeadingNotDesired).whileTrue(new BaseDriveCommand(o2s, operatorInput, swerveSubsystem));
        operatorInput.movementDesired.and(operatorInput.ampHeadingHold).whileTrue(new DriveFacingStaticPosCommand(o2s, operatorInput, swerveSubsystem, Rotation2d.fromDegrees(90), false, DFSP));
        operatorInput.movementDesired.and(operatorInput.rightSpeakerHeadingHold).whileTrue(new DriveFacingStaticPosCommand(o2s, operatorInput, swerveSubsystem, Rotation2d.fromRadians(Math.PI / 3), true, DFSP));
        operatorInput.movementDesired.and(operatorInput.leftSpeakerHeadingHold).whileTrue(new DriveFacingStaticPosCommand(o2s, operatorInput, swerveSubsystem, Rotation2d.fromRadians(-Math.PI / 3), true, DFSP));
        operatorInput.movementDesired.and(operatorInput.frontSpeakerHeadingHold).whileTrue(new DriveFacingStaticPosCommand(o2s, operatorInput, swerveSubsystem, Rotation2d.fromDegrees(0), true, DFSP));


        //OPERATOR STUFF
        operatorInput.isTeleop.and(operatorInput.climberThreshold).whileTrue(new MoveClimberCommand(climberSubsystem, operatorInput));
        operatorInput.rev.whileTrue(new SpinFlywheelIndefinite(flywheelSubsystem, false, COMMANDS.ramFireSpeed_mechanismRotationsPerSecond()));
        operatorInput.ampShotSpeed.whileTrue(new AmpMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, 12));
        operatorInput.groundIntakeNoBeamBreak.whileTrue(new BasicGroundIntakeCommand(groundIntakeSubsystem, noteManagementSubsystem, COMMANDS.groundIntake_voltage(), COMMANDS.noteManagement_voltage() ));
        operatorInput.shootManually.whileTrue(new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, COMMANDS.ramFireSpeed_mechanismRotationsPerSecond()));

        operatorInput.sourceIntake_hold.whileTrue(new SourceConsumerGroup(noteManagementSubsystem, flywheelSubsystem));
        operatorInput.groundIntakeHoldOp //TODO change input
                .whileTrue(new FeedGroundIntakeGroup(noteManagementSubsystem, groundIntakeSubsystem));
        operatorInput.groundOuttakeHoldOp
                .whileTrue(new GroundOuttakeCommand(groundIntakeSubsystem, noteManagementSubsystem));

        //zero heading, not anything else
        operatorInput.resetGyroPress.onTrue(new PlaceAllianceZeroHeading(odometry,Rotation2d.fromDegrees(0)));
    }


    Translation2d[] loadTranslations() {
        return new Translation2d[] {
                ODO.fl_offset(),
                ODO.fr_offset(),
                ODO.bl_offset(),
                ODO.br_offset()
        };
    }


    SwerveDriveKinematics loadKinematics() {
        return new SwerveDriveKinematics(
                translation2ds
        );
    }

    Modules loadModules() {
        SwerveModule[] modules = loadSwerveModules();

        return new Modules(modules,MODULES);
    }

    SwerveModule[] loadSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];


        for (int i = 0; i < modules.length; i++) {
            SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_SWERVE[i].ff_ks(), FF_SWERVE[i].ff_kv());
            ILinearVelocityController driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;

            if (DISABLER.drive_disabled()) {
                driveMotor = HardwareDisabled.linearMotor_velocityPID();
                steerController = HardwareDisabled.rotationalController_disabled();
                absoluteEncoder = HardwareDisabled.rotationEncoder_disabled();
            } else if (Robot.isSimulation()) {
                driveMotor = HardwareSIM.linearSIM_velocityPid(DRIVES[i], DRIVE_PIDS[i], DCMotor.getNEO(1));
                steerController = HardwareSIM.rotationalSIM_pid(STEERS[i], STEER_PIDS[i], DCMotor.getNEO(1));
                absoluteEncoder = steerController; //TODO silly hack wtf this is not a hack i have spent two hours on this and i have not found a solution
            } else {
                if (MattlibSettings.ROBOT == MattlibSettings.Robot.CARY) {
                    driveMotor = HardwareCTRE.linearFX_builtInVelocityPID(DRIVES[i], DRIVE_PIDS[i]);
                    absoluteEncoder = HardwareUtil.thriftyEncoder(STEER_ABS_ENCODERS[i]);
                    steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], STEER_PIDS[i]);
                } else {
                    driveMotor = HardwareREV.linearSpark_builtInVelocityPID(DRIVES[i], DRIVE_PIDS[i]);
                    steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], STEER_PIDS[i]);
                    absoluteEncoder = HardwareUtil.thriftyEncoder(STEER_ABS_ENCODERS[i]);
                }

            }
            /*
            if (disabled) {
                driveMotor = HardwareDisabled.linearMotor_disabled();
            }
*/
            modules[i] = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);
        }

        return modules;
    }

    FlywheelSubsystem loadFlywheelSubsystem() {
        IRotationalVelocityController leftMotor;
        IRotationalVelocityController rightMotor;

        IRotationEncoder velocityEncoderRight;
        IRotationEncoder velocityEncoderLeft;

        if (DISABLER.flywheel_disabled()) {
            leftMotor = HardwareDisabled.rotationalVelocityController_disabled();
            rightMotor = HardwareDisabled.rotationalVelocityController_disabled();
            velocityEncoderLeft = HardwareDisabled.rotationEncoder_disabled();
            velocityEncoderRight = HardwareDisabled.rotationEncoder_disabled();
        } else if (Robot.isSimulation()) {
            leftMotor = HardwareDisabled.rotationalVelocityController_disabled();//HardwareSIM.rotationalSIM_pid(SHOOTER_FLYWHEEL_LEFT, FLYWHEEL_VELOCITY_PID, DCMotor.getNEO(1));
            rightMotor = HardwareDisabled.rotationalVelocityController_disabled();//HardwareSIM.rotationalSIM_pid(SHOOTER_FLYWHEEL_RIGHT, FLYWHEEL_VELOCITY_PID, DCMotor.getNEO(1));
            velocityEncoderLeft = leftMotor; //TODO switch out leftMotor with actual velocity encoder
            velocityEncoderRight = rightMotor;

        } else {
            leftMotor = HardwareREV.rotationalSpark_builtInVelocityPID(SHOOTER_FLYWHEEL_LEFT, FLYWHEEL_VELOCITY_PID);
            rightMotor = HardwareREV.rotationalSpark_builtInVelocityPID(SHOOTER_FLYWHEEL_RIGHT, FLYWHEEL_VELOCITY_PID);
            velocityEncoderLeft = HardwareUtil.throughboreEncoder(FLYWHEEL_ENCODER_LEFT);
            velocityEncoderRight = HardwareUtil.throughboreEncoder(FLYWHEEL_ENCODER_RIGHT);
        }


        return new FlywheelSubsystem(
                leftMotor,
                rightMotor,
                FLYWHEEL,
                velocityEncoderLeft,
                velocityEncoderRight
        );

    }

    NoteManagementSubsystem loadNoteManagementSubsystem() {
        if (DISABLER.nms_disabled()) {
            return MockingUtil.buddy(NoteManagementSubsystem.class);
        }

        ILinearMotor nms_bottomMotor;
        ILinearMotor nms_topMotor;
        DigitalInput beamBreak = new DigitalInput(NMS.channel());

        if (DISABLER.nms_disabled()) {
            nms_bottomMotor = HardwareDisabled.linearController_disabled();
            nms_topMotor = HardwareDisabled.linearMotor_disabled();
        } else if (Robot.isSimulation()) {
            nms_bottomMotor = HardwareSIM.linearSIM_noPID(NMS_BOTTOMCOMPONENT, DCMotor.getNEO(1));
            nms_topMotor = HardwareSIM.linearSIM_noPID(NMS_TOPCOMPONENT, DCMotor.getNEO(1));
        } else {
            nms_bottomMotor = HardwareREV.linearSpark_noPID(NMS_BOTTOMCOMPONENT);
            nms_topMotor = HardwareREV.linearSpark_noPID(NMS_TOPCOMPONENT);
        }

        return new NoteManagementSubsystem(
                nms_bottomMotor, nms_topMotor, beamBreak,
                NMS);

    }

    Odometry loadOdometrySubsystem() {

        IGyro gyro;

        if (Robot.isSimulation() || DISABLER.odometry_disabled()) {
            gyro = new KinematicGyro(modules, kinematics);
        } else {
            Pigeon2 pigeon2 = new Pigeon2(ODO.pigeonCanId());

            gyro = new Pigeon2Gyro(pigeon2);
        }

        // TODO implement swappable version per condition (sim, disable, enable)

        return new Odometry(
                modules,
                visionSubsystem,
                new SwerveDrivePoseEstimator( //The auto path will reset all of this data anyways
                        kinematics,
                        new Rotation2d(),
                        new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
                        new Pose2d()
                ),
                gyro,
                kinematics,
                ODO
        ); //TODO
    }

    VisionSubsystem loadVisionSubsystem() {
        if (DISABLER.vision_disabled()) {
            return MockingUtil.buddy(VisionSubsystem.class);
        }

        PhotonCamera camera1;
        PhotonCamera camera2;
        camera1 = new PhotonCamera(CAMERAS.camera1Name());
        camera2 = new PhotonCamera(CAMERAS.camera2Name());

        AprilTagFieldLayout aprilTagFieldLayout;
        String filepath = Filesystem.getDeployDirectory().getPath() + "/2024-crescendo.json";

        // better error catching later ig
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(filepath);
        } catch (IOException e) {
            throw new IllegalStateException(e.getMessage() + " here is why");
        }

        // USE MATTLIB FOR CONSTANTS HERE IM JUST LAZY TODO
        Transform3d robotToCam1 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        // using multi tag localization
        PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                robotToCam1
        );


        Transform3d robotToCam2 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera2,
                robotToCam2
        );

        photonPoseEstimator1.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        return new VisionSubsystem(
                camera1,
                camera2,
                aprilTagFieldLayout,
                photonPoseEstimator1,
                photonPoseEstimator2,
                new AprilTagDetector()
        );

    }

    ClimberSubsystem loadClimberSubsystem() {
        ILinearController leftClimber;
        ILinearController rightClimber;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(CLIMBER.ff_ks(), CLIMBER.ff_kv());
        if (DISABLER.climber_disabled()) {
            leftClimber = HardwareDisabled.linearController_disabled();
            rightClimber = HardwareDisabled.linearController_disabled();
        } else {
            leftClimber = HardwareREV.linearSpark_builtInPID(LEFT_CLIMBER, CLIMBER_PID);
            rightClimber = HardwareREV.linearSpark_builtInPID(RIGHT_CLIMBER, CLIMBER_PID);
        }

        return new ClimberSubsystem(
                leftClimber,
                rightClimber,
                feedForward
        ); // TODO

    }

    GroundIntakeSubsystem loadGroundIntakeSubsystem() {
        ILinearController topGroundIntake;
        ILinearController bottomGroundIntake;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(GROUND_INTAKE.ff_ks(), GROUND_INTAKE.ff_kv());

        if (DISABLER.groundIntake_disabled()) {
            topGroundIntake = HardwareDisabled.linearController_disabled();
            bottomGroundIntake = HardwareDisabled.linearController_disabled();

        } else if (Robot.isSimulation()) {
            topGroundIntake = HardwareSIM.linearSIM_pid(GROUND_INTAKE_TOP, GROUND_INTAKE_PID, DCMotor.getNeo550(1));
            bottomGroundIntake = HardwareSIM.linearSIM_pid(GROUND_INTAKE_BOTTOM, GROUND_INTAKE_PID, DCMotor.getNeo550(1));
        } else {
            topGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_TOP, GROUND_INTAKE_PID);
            bottomGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_BOTTOM, GROUND_INTAKE_PID);
        }

        return new GroundIntakeSubsystem(
                topGroundIntake,
                bottomGroundIntake,
                feedForward
        );
    }

    public void robotInit() {

    }


    //Components MUST be created in the Robot class (because of how static bs works)
    static final GenerateFFComponent LEFT_FLYWHEEL_GEN = LOG.load(GenerateFFComponent.class, "leftFlywheelGen");
    static final GenerateFFComponent RIGHT_FLYWHEEL_GEN = LOG.load(GenerateFFComponent.class, "rightFlywheelGen");


    //generator stuff
    public static final ConsoleComponent CONSOLE = LOG.load(ConsoleComponent.class, "console");

    //commands and autp
    public static final CommandComponent COMMANDS = LOG.load(CommandComponent.class, "commands");
    public static final DriveFacingStaticPosCommand.Component DFSP = LOG.load(DriveFacingStaticPosCommand.Component.class, "commands/dfsp");

    //vision
    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    //climber
    public static final ClimberComponent CLIMBER = LOG.load(ClimberComponent.class, "climber");
    public static final PIDComponent CLIMBER_PID = LOG.load(PIDComponent.class, "climber/climber_pid");
    public static final MotorComponent LEFT_CLIMBER = LOG.load(MotorComponent.class, "climber/left");
    public static final MotorComponent RIGHT_CLIMBER = LOG.load(MotorComponent.class, "climber/right");

    //ground intake
    public static final GroundIntakeComponent GROUND_INTAKE = LOG.load(GroundIntakeComponent.class, "ground_intake");
    public static final MotorComponent GROUND_INTAKE_TOP = LOG.load(MotorComponent.class, "ground_intake/top");
    public static final MotorComponent GROUND_INTAKE_BOTTOM = LOG.load(MotorComponent.class, "ground_intake/bottom");
    public static final PIDComponent GROUND_INTAKE_PID = LOG.load(PIDComponent.class, "ground_intake/pid");

    //flywheel
    public static final FlywheelSubsystem.ShooterComponent FLYWHEEL = LOG.load(FlywheelSubsystem.ShooterComponent.class, "flywheel");
    public static final MotorComponent SHOOTER_FLYWHEEL_LEFT = LOG.load(MotorComponent.class, "flywheel/left");
    public static final MotorComponent SHOOTER_FLYWHEEL_RIGHT = LOG.load(MotorComponent.class, "flywheel/right");
    public static final PIDComponent FLYWHEEL_VELOCITY_PID = LOG.load(PIDComponent.class, "flywheel/velocity_pid");
    public static final DigitalEncoderComponent FLYWHEEL_ENCODER_LEFT = LOG.load(DigitalEncoderComponent.class, "flywheel/left/encoder");
    public static final DigitalEncoderComponent FLYWHEEL_ENCODER_RIGHT = LOG.load(DigitalEncoderComponent.class, "flywheel/right/encoder");
    public static final FFComponent FLYWHEEL_LEFT_FF = LOG.load(FFComponent.class, "flywheel/left/ff");
    public static final FFComponent FLYWHEEL_RIGHT_FF = LOG.load(FFComponent.class, "flywheel/right/ff");

    //note management
    public static final NoteManagementComponent NMS = LOG.load(NoteManagementComponent.class, "nms");
    public static final MotorComponent NMS_TOPCOMPONENT = LOG.load(MotorComponent.class, "nms/top");
    public static final MotorComponent NMS_BOTTOMCOMPONENT = LOG.load(MotorComponent.class, "nms/bottom");

    //swerve
    public static final FFComponent[] FF_SWERVE = LOG.loadRange(FFComponent.class, "swerve/ff", 4, Util.RENAMER);
    public static final OperatorToSpeeds.Component O2S = LOG.load(OperatorToSpeeds.Component.class, "operatorToSpeeds");
    public static final DriveSubsystem.Component SWERVE = LOG.load(DriveSubsystem.Component.class, "swerve");
    public static final Odometry.Component ODO = LOG.load(Odometry.Component.class, "odometry");
    public static final Modules.Component MODULES = LOG.load(Modules.Component.class, "modules");
    public static final CommonMotorComponent STEER_COMMON = LOG.load(CommonMotorComponent.class, "swerve/steer_common");
    public static final CommonPIDComponent STEER_PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/steer_pid_common");
    public static final CommonPIDComponent DRIVE_PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/drive_pid_common");

    //swerve2
    public static final MotorComponent[] DRIVES = LOG.loadRange(MotorComponent.class, "swerve/drive", 4, Util.RENAMER);
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(STEER_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER));
    public static final PIDComponent[] STEER_PIDS = PIDComponent.ofRange(STEER_PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/steer_pid", 4, Util.RENAMER));
    public static final PIDComponent[] DRIVE_PIDS = PIDComponent.ofRange(DRIVE_PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/drive_pid", 4, Util.RENAMER));

    public static final AnalogEncoderComponent[] STEER_ABS_ENCODERS = LOG.loadRange(AnalogEncoderComponent.class, "swerve/abs", 4, Util.RENAMER);

    public static final PIDComponent DRIVE_X_PID = LOG.load(PIDComponent.class, "swerve/x_holonomic_pid");
    public static final PIDComponent DRIVE_Y_PID = LOG.load(PIDComponent.class, "swerve/y_holonomic_pid");
    public static final PIDComponent DRIVE_T_PID = LOG.load(PIDComponent.class, "swerve/t_holonomic_pid");

    public static final CamerasComponent CAMERAS = LOG.load(CamerasComponent.class, "cameras");
    public static final DisablerComponent DISABLER = LOG.load(DisablerComponent.class, "disabler");


}

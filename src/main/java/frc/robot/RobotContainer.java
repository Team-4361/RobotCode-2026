// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.logics.SnapToHubCommand;
import frc.robot.logics.Vision;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import frc.robot.util.FuelSim;



public class RobotContainer
{
   //shooter hell yeah twin 
   public static final double SHOOTER_REV_SPEED = 0.6;

    

    // ========== FIELD CONSTANTS ==========
    private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);
    private static final double FIELD_WIDTH_M = Units.inchesToMeters(317.69);

    public static final Translation2d HUB_CENTER_BLUE =

        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

    public static final Translation2d HUB_CENTER_RED =
        new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                          Units.inchesToMeters(158.84));

        Field2d fullPose  = new Field2d();

    // ========== CONTROLLERS ==========
    final CommandJoystick joystickL     = new CommandJoystick(0);
    final CommandJoystick joystickR     = new CommandJoystick(1);
    double xV = 0;
    double yV = 0;
    double rV = 0;
    final CommandXboxController operatorXbox = new CommandXboxController(2);
    final CommandXboxController testXbox     = new CommandXboxController(3);

    // ========== SUBSYSTEMS ==========
    public final static SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private final Vision vision = new Vision(drivebase);

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);


private Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            FIELD_LENGTH_M - pose.getX(),
            Math.abs(FIELD_WIDTH_M - pose.getY())
        ),
        pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
    );
}
private Pose2d getShootTarget() {
    Pose2d redTarget = new Pose2d(new Translation2d(15.483717765710468, 5.278244), Rotation2d.fromDegrees(-85));
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        return mirrorPose(redTarget);
    }
    return redTarget;
}

//    private final Command teleopFlightDriveCommand = drivebase.driveFieldOriented(
//     SwerveInputStream.of(
//         drivebase.getSwerveDrive(),
//         () -> -joystickL.getY(),  // Forward/Backward
//         () -> -joystickL.getX()   // Left/Right
//     )
//     .withControllerRotationAxis(() -> -joystickR.getTwist() * 0.95) // Rotation using right stick twist
//     .deadband(OperatorConstants.DEADBAND) // Apply deadband as a setting
//     .scaleTranslation(0.8)
//     .allianceRelativeControl(true)
// );


    private final AgitatorSubsystem agitator = new AgitatorSubsystem();
    private final IndexerSubsystem  indexer  = new IndexerSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeArmSubsystem    intakeArm    = new IntakeArmSubsystem();
    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    public FuelSim fuelSim;


    private SendableChooser<Command> autoChooser;

    // private final Command teleopFlightDriveCommand = drivebase.driveFieldOriented(
    //     SwerveInputStream.of(drivebase.getSwerveDrive(), () -> xV, () -> yV)
    //                      .withControllerRotationAxis(() -> rV)
    //                      .deadband(OperatorConstants.DEADBAND)
    //                      .scaleTranslation(0.8)
    //                      .allianceRelativeControl(true));

    public RobotContainer()
    {
        SmartDashboard.putNumber("INDEXER_SPEED",  1.0);
        SmartDashboard.putNumber("INDEXER_SPEED",  0.9);
        SmartDashboard.putNumber("SHOOTER_SPEED", 0.77);
        SmartDashboard.putNumber("INTAKE_SPEED",  0.7);
        shooter.setDefaultCommand(shooter.STOPP());
        indexer.setDefaultCommand(indexer.stopMotorCommand());
        agitator.setDefaultCommand(agitator.stopMotorCommand());

        registerNamedCommands();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
                if (Robot.isSimulation()) {
            configureFuelSim();
        }

        // FIX: Use RobotModeTriggers.autonomous() unconditionally.
        // The old null-check on autoChooser.getSelected() was unreliable —
        // getSelected() at construction time always returns the default anyway.
        // We always want to zero+re-seed at auto init regardless.
        RobotModeTriggers.autonomous().onTrue(
            Commands.runOnce(this::zeroGyroAndReseed)
        );
    }

    /**
     * Gives other classes (e.g. Robot's teleopController, which is built
     * after RobotContainer so this subsystem already exists) access to the
     * shooter without exposing every subsystem or making them all static.
     */
    public ShooterSubsystem getShooter()
    {
        return shooter;
    }

    //SIMULATION//

    private void configureFuelSim() {
    fuelSim = new FuelSim("fuel");
    fuelSim.spawnStartingFuel();

    // Convert robot-relative speeds to field-relative using current heading
    fuelSim.registerRobot(
        Units.inchesToMeters(31), // width  (bumper to bumper)
        Units.inchesToMeters(31), // length (bumper to bumper)
        Units.inchesToMeters(5.1), // bumper height — adjust if needed
        drivebase::getPose,
        () -> {
            ChassisSpeeds robotSpeeds = drivebase.getRobotVelocity();
            Rotation2d heading = drivebase.getPose().getRotation();
            return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);
        }
    );

    // Intake bounding box — robot-centric, roughly front-right side
    // Adjust minX/maxX/minY/maxY to match your actual intake geometry
    fuelSim.registerIntake(
    Units.inchesToMeters(-16.5),  // minX — rear bumper edge
    Units.inchesToMeters(-8),   // maxX — a few inches in from the back
    Units.inchesToMeters(-11),  // minY — right side
    Units.inchesToMeters(11),   // maxY — left side
               () -> System.out.println("FUEL INTAKED!") // debug callback                    // optional callback e.g. hopper.increment()
    );

    fuelSim.setSubticks(5);
    fuelSim.start();

    // SmartDashboard reset button
    SmartDashboard.putData("Reset Fuel",
        Commands.runOnce(() -> {
            fuelSim.clearFuel();
            fuelSim.spawnStartingFuel();
        }).ignoringDisable(true).withName("Reset Fuel")
    );
}




    // =========================================================================
    // VISION UPDATE — call this from Robot.robotPeriodic() every loop
    // =========================================================================
    public void updateVision()
    {

        vision.updatePhotonVision();

    }


    private void zeroGyroAndReseed()
    {
        drivebase.zeroGyroWithAlliance();


    }

    // =========================================================================
    // SHOOT COMMAND FACTORY
    // =========================================================================
    private Command shootWithFeedCommand()
    {
        return Commands.defer(() ->
            Commands.sequence(
                shooter.setSPEED(SHOOTER_REV_SPEED).alongWith(
                ).withTimeout(0),
                shooter.setSPEED(SHOOTER_REV_SPEED)
                       .alongWith(
                           agitator.runMotorCommand(0.3),
                           indexer.runMotorCommand(-0.4)
                       )
            ).withName("ShootWithFeed"),
            Set.of(shooter, agitator, indexer)
        );
    }

    private Command shootWithFeedCommandAuto(double shooterSpeed, double durationSeconds)
    {
        return Commands.defer(() ->
            Commands.sequence(
                // Spin up shooter first
                shooter.setSPEED(SHOOTER_REV_SPEED)
                    .withTimeout(0.0),

                // Run shooter + hopper + feeder together for the given duration
                shooter.setSPEED(SHOOTER_REV_SPEED)
                    .alongWith(
                           agitator.runMotorCommand(0.3),
                           indexer.runMotorCommand(-0.4)
                    )
                    .withTimeout(durationSeconds),

                // Stop all subsystems cleanly
                shooter.STOPP()
                    .alongWith(
                        agitator.runMotorCommand(0),
                        indexer.runMotorCommand(0)
                    )
                    .withTimeout(0.1)
            ).withName("ShootWithFeed"),
            Set.of(shooter, agitator, indexer)
        );
    }

private Command shootWithIntakeBounceAuto(double shooterSpeed, double durationSeconds)
{
    return Commands.defer(() ->
        Commands.sequence(
            // Spin up shooter first
            shooter.set(shooterSpeed)
                .withTimeout(0.35),

            // Shoot + agitator + indexer + intake bouncing for duration
            shooter.set(shooterSpeed)
                .alongWith(
                    agitator.runMotorCommand(SmartDashboard.getNumber("AGITATOR_SPEED", 0.9)),
                    indexer.runMotorCommand(SmartDashboard.getNumber("INDEXER_SPEED", 0.9)),
                    Commands.repeatingSequence(
                        intakeArm.stowCommand(),
                        intakeArm.deployCommand()
                    )
                )
                .withTimeout(durationSeconds),

            // Stop shooter/agitator/indexer, deploy intake down at the end
            shooter.set(0)
                .alongWith(
                    agitator.runMotorCommand(0),
                    indexer.runMotorCommand(0),
                    intakeArm.deployCommand()
                )
                .withTimeout(0.1)
        ).withName("ShootWithIntakeBounce"),
        Set.of(shooter, agitator, indexer, intakeArm)
    );
}



    private void registerNamedCommands()
    {
        NamedCommands.registerCommand("SnapToHub",   new SnapToHubCommand());

       
        //NamedCommands.registerCommand("ShooterSpinUp", shooter.aimAtHubContinuous(drivebase));
// NamedCommands.registerCommand("DeployIntake",
//     Commands.defer(() -> intakeArm.deploySimple(), Set.of(intakeArm))
// );        


        NamedCommands.registerCommand("DeployIntake",intakeArm.deployCommand());
        NamedCommands.registerCommand("stowHalf",intakeArm.stowHalfCommand());

NamedCommands.registerCommand("stowIntake",
    Commands.defer(() -> intakeArm.stowCommand(), Set.of(intakeArm))
);
        NamedCommands.registerCommand("Intake", intakeRoller.intakeCommand() );
        NamedCommands.registerCommand("StopIntake", 
            Commands.runOnce(intakeRoller::stop, intakeRoller)
        );

        NamedCommands.registerCommand("ShooterStop",   Commands.runOnce(() -> shooter.set(0), shooter));
        NamedCommands.registerCommand("Shoot",         shootWithFeedCommandAuto(0.77, 6.7));
        NamedCommands.registerCommand("ShootCenter",         shootWithFeedCommandAuto(SHOOTER_REV_SPEED, 3));
        NamedCommands.registerCommand("ShootFull",         shootWithFeedCommand());

        NamedCommands.registerCommand("RevShooter", shooter.revShooter(SHOOTER_REV_SPEED));
    NamedCommands.registerCommand("ShootBounce", shootWithIntakeBounceAuto(0.77, 6.7));
// NamedCommands.registerCommand("stowHalf",
//     Commands.defer(() -> intakeArm.stowHalfCommand(), Set.of(intakeArm))
// );        
        
        
        
        

    }

    private void configureBindings()
    {

        // ── Simulation bindings ───────────────────────────────────────────────
        if (Robot.isSimulation())
        {
            // FIX: Use zeroGyroAndReseed() instead of raw zeroGyro() so vision
            // re-seeds after every manual zero, even in sim.

            //15.483717765710468, 5.278244, -85
            testXbox.a().whileTrue(Commands.runOnce(this::zeroGyroAndReseed));
            testXbox.b().whileTrue(Commands.runOnce(this::zeroGyroAndReseed));

        }

        if (DriverStation.isTest())
        {
            // test mode bindings here if needed
        }
        else
        {


        joystickL.button(2).whileTrue(
            Commands.defer(() -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                Pose2d target;
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    target = new Pose2d(new Translation2d(15.483717765710468, 5.278244), Rotation2d.fromDegrees(-85));
                } else {
                    // Mirror X across the field for blue alliance
                    target = new Pose2d(new Translation2d(FIELD_LENGTH_M - 15.483717765710468, 5.278244), Rotation2d.fromDegrees(-95));
                }
                return drivebase.driveToPose(target);
            }, Set.of(drivebase))
        );
            // ── Driver bindings ──────────────────────────────────────────────────

            // FIX: Was drivebase::zeroGyro (no alliance, no vision re-seed).
            // Now calls zeroGyroAndReseed() which handles all three steps.
            joystickR.button(5).onTrue(Commands.runOnce(this::zeroGyroAndReseed));


// drivebase.setDefaultCommand(teleopFlightDriveCommand);

            
            
            operatorXbox.a().onTrue(intakeArm.deployCommand());
            operatorXbox.b().onTrue(intakeArm.stowCommand());
            operatorXbox.x().onTrue(intakeArm.stowHalfCommand());
            operatorXbox.rightTrigger(0.3).toggleOnTrue(intakeRoller.intakeCommand());
        operatorXbox.leftTrigger(0.3).whileTrue(intakeRoller.outtakeCommand());
        operatorXbox.povUp().whileTrue(intakeArm.manualCommand());
        operatorXbox.povDown().whileTrue(intakeArm.manualCommandDown());


            // ── Operator: Shoot ──────────────────────────────────────────────────
            operatorXbox.rightBumper().whileTrue(shootWithFeedCommand());
            operatorXbox.leftBumper().whileTrue(shooter.setSPEED(SHOOTER_REV_SPEED));

            
        }
    }

    private Translation2d getHubTarget()
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return HUB_CENTER_RED;
        }
        return HUB_CENTER_BLUE;
    }

    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake)
    {
        drivebase.setMotorBrake(brake);
    }
}
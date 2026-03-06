// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.logics.Vision;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

import java.io.File;
import java.util.Optional;
import java.util.Set;

import swervelib.SwerveInputStream;

public class RobotContainer
{

    // ========== FIELD CONSTANTS ==========
    private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);

    public static final Translation2d HUB_CENTER_BLUE =
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

    public static final Translation2d HUB_CENTER_RED =
        new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                          Units.inchesToMeters(158.84));

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

    private final IntakeSubsystem  intake  = new IntakeSubsystem();
    private final HopperSubsystem  hopper  = new HopperSubsystem();
    private final FeederSubsystem  feeder  = new FeederSubsystem();
    private final TurretSubsystem  turret  = new TurretSubsystem(drivebase);
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private static final double TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC = 90.0;

    private SendableChooser<Command> autoChooser;

    // private final Command teleopFlightDriveCommand = drivebase.driveFieldOriented(
    //     SwerveInputStream.of(drivebase.getSwerveDrive(), () -> xV, () -> yV)
    //                      .withControllerRotationAxis(() -> rV)
    //                      .deadband(OperatorConstants.DEADBAND)
    //                      .scaleTranslation(0.8)
    //                      .allianceRelativeControl(true));

    public RobotContainer()
    {
        SmartDashboard.putNumber("HOPPER_SPEED",  1.0);
        SmartDashboard.putNumber("FEEDER_SPEED",  0.9);
        SmartDashboard.putNumber("SHOOTER_SPEED", 0.9);
        SmartDashboard.putNumber("INTAKE_SPEED",  0.7);

        shooter.setDefaultCommand(shooter.set(0));
        hopper.setDefaultCommand(hopper.stopMotorCommand());
        feeder.setDefaultCommand(feeder.stopMotorCommand());

        registerNamedCommands();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // FIX: Use RobotModeTriggers.autonomous() unconditionally.
        // The old null-check on autoChooser.getSelected() was unreliable —
        // getSelected() at construction time always returns the default anyway.
        // We always want to zero+re-seed at auto init regardless.
        RobotModeTriggers.autonomous().onTrue(
            Commands.runOnce(this::zeroGyroAndReseed)
        );
    }

    // =========================================================================
    // VISION UPDATE — call this from Robot.robotPeriodic() every loop
    // =========================================================================
    public void updateVision()
    {
        vision.updateVision();
    }

    // =========================================================================
    // SAFE GYRO ZERO
    //
    // This is the ONLY method that should be called to zero the gyro.
    // It handles three things in order:
    //   1. Zero the gyro (NavX heading → 0)
    //   2. Apply alliance offset (red alliance robots face 180°, blue face 0°)
    //   3. Tell vision to re-seed odometry from the next good AprilTag reading,
    //      because after a gyro zero the stored odometry heading is now wrong
    //      relative to what vision expects.
    //
    // WHY re-seed?
    //   After zeroGyroWithAlliance(), the X/Y in odometry is still wherever
    //   the robot physically is, but vision's hasSeededOdometry=true means it
    //   will run the poseDifference check. If the robot moved since last seed
    //   that diff could be fine — but the heading is now authoritative from
    //   the gyro, not vision, so we just let vision re-confirm X/Y on the
    //   next good tag read without resetting the seed flag entirely.
    //   Actually, we DO reset the seed flag so vision hard-resets X/Y too,
    //   because after a gyro zero the driver has declared "I know where I am"
    //   and vision should confirm it ASAP.
    // =========================================================================
    private void zeroGyroAndReseed()
    {
        // Step 1 & 2: zero gyro with correct alliance heading
        drivebase.zeroGyroWithAlliance();

        // Step 3: tell vision to re-seed X/Y from the next good tag reading.
        // This clears hasSeededOdometry so the diff check is skipped once,
        // letting vision hard-reset position without fighting the old odometry.
        vision.resetSeedFlag();
    }

    // =========================================================================
    // SHOOT COMMAND FACTORY
    // =========================================================================
    private Command shootWithFeedCommand()
    {
        return Commands.defer(() ->
            Commands.sequence(
                shooter.set(SmartDashboard.getNumber("SHOOTER_SPEED", 0.9))
                       .withTimeout(0.35),
                shooter.set(SmartDashboard.getNumber("SHOOTER_SPEED", 0.9))
                       .alongWith(
                           hopper.runMotorCommand(SmartDashboard.getNumber("HOPPER_SPEED", 1.0)),
                           feeder.runMotorCommand(SmartDashboard.getNumber("FEEDER_SPEED", 0.9))
                       )
            ).withName("ShootWithFeed"),
            Set.of(shooter, hopper, feeder)
        );
    }

        private Command shootWithFeedCommandAuto(double shooterspeed)
    {
        return Commands.defer(() ->
            Commands.sequence(
                shooter.set(shooterspeed)
                       .withTimeout(0.35),
                shooter.set(shooterspeed)
                       .alongWith(
                           hopper.runMotorCommand(SmartDashboard.getNumber("HOPPER_SPEED", 1.0)),
                           feeder.runMotorCommand(SmartDashboard.getNumber("FEEDER_SPEED", 0.9))
                       )
            ).withName("ShootWithFeed"),
            Set.of(shooter, hopper, feeder)
        );
    }

    private void registerNamedCommands()
    {
        NamedCommands.registerCommand("SetTurretAngle_0",   turret.setAngleCommand(0.0));
        NamedCommands.registerCommand("SetTurretAngle_0",   turret.setAngleCommand(62.5));

        NamedCommands.registerCommand("SetTurretAngle_90",  turret.setAngleCommand(90.0));
        NamedCommands.registerCommand("SetTurretAngle_180", turret.setAngleCommand(180.0));
        NamedCommands.registerCommand("SetTurretAngle_-90", turret.setAngleCommand(-90.0));
        NamedCommands.registerCommand("TurretAimForward",   turret.moveToAngleCommand(0.0).withTimeout(3.0));
        NamedCommands.registerCommand("TurretAimHub",       turret.aimAtTargetCommand(getHubTarget()).withTimeout(3.0));

        NamedCommands.registerCommand("runIntake",   intake.runMotorCommand(0.5));
        NamedCommands.registerCommand("stopIntake",  intake.stopMotorCommand());

        NamedCommands.registerCommand("ShooterSpinUp", shooter.aimAtHubContinuous(drivebase));
        NamedCommands.registerCommand("ShooterStop",   Commands.runOnce(() -> shooter.set(0), shooter));
        NamedCommands.registerCommand("Shoot",         shootWithFeedCommand());
        NamedCommands.registerCommand("Shoot",         shootWithFeedCommand());

    }

    private void configureBindings()
    {
        turret.setDefaultCommand(turret.holdPositionCommand());

        // ── Simulation bindings ───────────────────────────────────────────────
        if (Robot.isSimulation())
        {
            // FIX: Use zeroGyroAndReseed() instead of raw zeroGyro() so vision
            // re-seeds after every manual zero, even in sim.
            testXbox.a().whileTrue(Commands.runOnce(this::zeroGyroAndReseed));
            testXbox.b().whileTrue(Commands.runOnce(this::zeroGyroAndReseed));

            Pose2d target = new Pose2d(new Translation2d(2.772, 4.062), Rotation2d.fromDegrees(0));
            testXbox.x().whileTrue(drivebase.driveToPose(target));
        }

        if (DriverStation.isTest())
        {
            // test mode bindings here if needed
        }
        else
        {
            // ── Driver bindings ──────────────────────────────────────────────────

            // FIX: Was drivebase::zeroGyro (no alliance, no vision re-seed).
            // Now calls zeroGyroAndReseed() which handles all three steps.
            joystickL.button(12).onTrue(Commands.runOnce(this::zeroGyroAndReseed));

            joystickL.button(5).onTrue(Commands.runOnce(
                () -> {
                    drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)));
                    // Re-seed after manual pose reset so vision confirms the new position
                    vision.resetSeedFlag();
                }
            ));

            joystickL.button(3).onTrue(Commands.runOnce(
                () -> {
                    drivebase.resetOdometry(new Pose2d(15.511, 6.537, new Rotation2d()));
                    vision.resetSeedFlag();
                }
            ));

            // ── Operator: Intake ─────────────────────────────────────────────────
            operatorXbox.a().whileTrue(intake.runMotorButBetter(0.85));
            operatorXbox.y().whileTrue(intake.runMotorButBetter(-0.85));

            // ── Operator: Turret ─────────────────────────────────────────────────
            operatorXbox.leftBumper().toggleOnTrue(
                turret.fieldAngleLockCommand().withName("TurretFieldLock"));

            operatorXbox.x().whileTrue(turret.manualControlCommand(() ->  1.0, TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC));
            operatorXbox.b().whileTrue(turret.manualControlCommand(() -> -1.0, TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC));

            operatorXbox.rightTrigger(0.5).toggleOnTrue(
                turret.aimAtTargetCommand(getHubTarget()).withName("TurretHubLock"));

            // ── Operator: Shoot ──────────────────────────────────────────────────
            operatorXbox.rightBumper().whileTrue(shootWithFeedCommand());
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
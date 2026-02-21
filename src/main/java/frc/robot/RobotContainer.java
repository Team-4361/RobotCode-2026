// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Hopper.HopperCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // ========== FIELD CONSTANTS ==========
  // Full 2026 field length in meters
  private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);

  // Hub/goal center positions used by the turret and shooter for targeting.
  // These are in field-relative coordinates (origin = blue alliance wall corner).
  public static final Translation2d HUB_CENTER_BLUE =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

  public static final Translation2d HUB_CENTER_RED =
      new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                        Units.inchesToMeters(158.84));

  // ========== CONTROLLERS ==========
  final CommandJoystick joystickL   = new CommandJoystick(0);
  final CommandJoystick joystickR   = new CommandJoystick(1);
  double xV = 0;
  double yV = 0;
  double rV = 0;
  final CommandXboxController driverXbox   = new CommandXboxController(2);
  // Operator controller — turret, shooter, and other mechanisms on port 3
  final CommandXboxController operatorXbox = new CommandXboxController(3);

  // ========== SUBSYSTEMS ==========
  public final static SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  SlewRateLimiter xfilter = new SlewRateLimiter(4);
  SlewRateLimiter yfilter = new SlewRateLimiter(4);
  SlewRateLimiter rfilter = new SlewRateLimiter(4);

  private final IntakeSubsystem  intake  = new IntakeSubsystem();
  private final HopperSubsystem  hopper  = new HopperSubsystem();
  private final TurretSubsystem  turret  = new TurretSubsystem(drivebase);
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // Vision is registered with the CommandScheduler automatically by extending SubsystemBase.
  // Its periodic() method feeds PhotonVision pose estimates into the swerve drive pose estimator
  // every loop — no manual calls needed anywhere else.
  private final Vision vision = new Vision(drivebase);

  // ========== TURRET CONFIG ==========
  private static final double TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC = 90.0;

  // ========== AUTO ==========
  private SendableChooser<Command> autoChooser;


  // ========== SWERVE INPUT STREAMS ==========

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                           .withControllerHeadingAxis(driverXbox::getRightX,
                                                                                      driverXbox::getRightY)
                                                           .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
                                                             .robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(
                                                                        () -> driverXbox.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                           .withControllerHeadingAxis(
                                                                               () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                               () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                                                           .headingWhile(true)
                                                                           .translationHeadingOffset(true)
                                                                           .translationHeadingOffset(Rotation2d.fromDegrees(0));

  private final Command teleopFlightDriveCommand = drivebase.driveFieldOriented(
      SwerveInputStream.of(drivebase.getSwerveDrive(), () -> xV, () -> yV)
                       .withControllerRotationAxis(() -> rV)
                       .deadband(OperatorConstants.DEADBAND)
                       .scaleTranslation(0.8)
                       .allianceRelativeControl(true));


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    registerNamedCommands();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null) {
      RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
  }


  private void registerNamedCommands()
  {
    // Turret angle presets
    NamedCommands.registerCommand("SetTurretAngle_0",   turret.setAngleCommand(0.0));
    NamedCommands.registerCommand("SetTurretAngle_90",  turret.setAngleCommand(90.0));
    NamedCommands.registerCommand("SetTurretAngle_180", turret.setAngleCommand(180.0));
    NamedCommands.registerCommand("SetTurretAngle_-90", turret.setAngleCommand(-90.0));

    // Aim turret at the hub and wait until on target (3s timeout)
    NamedCommands.registerCommand("TurretAimForward",
        turret.moveToAngleCommand(0.0).withTimeout(3.0));
    NamedCommands.registerCommand("TurretAimHub",
        turret.aimAtTargetCommand(getHubTarget()).withTimeout(3.0));

    // Shooter auto commands
    NamedCommands.registerCommand("ShooterSpinUp",  shooter.aimAtHubContinuous(drivebase));
    NamedCommands.registerCommand("ShooterStop",    Commands.runOnce(() -> shooter.set(0), shooter));
  }


  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle              = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity         = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity          = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    // ── Turret default command: right stick X = manual rotation ───────────
    turret.setDefaultCommand(
        turret.manualControlCommand(
            () -> operatorXbox.getRightX(),
            TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC));

    // ── Simulation bindings ───────────────────────────────────────────────
    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(5, 0, 0,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

      driverXbox.start().onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    // ── Test mode bindings ────────────────────────────────────────────────
    if (DriverStation.isTest())
    {
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
    else
    {
      // ── Driver bindings ──────────────────────────────────────────────────
      joystickL.button(12).onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.a().whileTrue(new IntakeCommand(intake, 0));
      driverXbox.b().whileTrue(new HopperCommand(hopper));
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      joystickL.button(5).onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)))));
      joystickL.button(3).onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(15.511, 6.537, new Rotation2d()))));

      // ── Operator: Turret bindings ─────────────────────────────────────────
      //
      // Face buttons snap turret to cardinal preset angles
      operatorXbox.a().onTrue(turret.setAngleCommand(0.0));    // Forward
      operatorXbox.b().onTrue(turret.setAngleCommand(90.0));   // Right
      operatorXbox.x().onTrue(turret.setAngleCommand(-90.0));  // Left
      operatorXbox.y().onTrue(turret.setAngleCommand(180.0));  // Backward

      // Left bumper: hard stop the turret motor
      operatorXbox.leftBumper().onTrue(turret.stopCommand());

      // Right bumper TOGGLE — hub lock mode.
      // First press: turret continuously tracks the alliance hub using field-relative math.
      // Second press: releases back to manual right-stick control.
      // Alliance is resolved at the moment the button is pressed via getHubTarget().
      operatorXbox.rightBumper().toggleOnTrue(
          turret.aimAtTargetCommand(getHubTarget())
                .withName("TurretHubLock"));

      // ── Operator: Shooter bindings ────────────────────────────────────────
      //
      // Left trigger (>50%) TOGGLE — shooter hub-aim mode.
      // First pull: flywheel spins up and continuously recalculates the exact RPM
      //   needed to score from the robot's current position, compensating for
      //   robot velocity.  RPM updates every loop.
      // Second pull: stops the flywheel.
      operatorXbox.leftTrigger(0.5).toggleOnTrue(
          shooter.aimAtHubContinuous(drivebase)
                 .withName("ShooterHubAim"));
    }
  }


  /**
   * Returns the correct hub Translation2d for the current alliance.
   * Safely falls back to blue if the alliance hasn't been set yet.
   */
  private Translation2d getHubTarget()
  {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return HUB_CENTER_RED;
    }
    return HUB_CENTER_BLUE;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
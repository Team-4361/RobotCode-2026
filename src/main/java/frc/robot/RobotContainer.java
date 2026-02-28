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
import frc.robot.logics.Vision;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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
  private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);

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
  final CommandXboxController operatorXbox = new CommandXboxController(3);

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

  // Vision auto-runs via its periodic() — no manual calls needed anywhere.
  //private final Vision vision = new Vision(drivebase);

  // ========== TURRET CONFIG ==========
  private static final double TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC = 90.0;

  // ========== FEED SYSTEM SPEEDS ==========
  private static final double HOPPER_SPEED = 0.5;
  private static final double FEEDER_SPEED = 0.5;

  // =========================================================================
  //  SHOOT COMMAND FACTORY
  //
  //  Rules:
  //    - Hopper and feeder MUST always run together with the shooter.
  //      Running hopper/feeder without the shooter will jam balls at
  //      the shooter wheel since there's nothing to clear them out.
  //    - Shooter CAN run alone (spin-up / warm-up before feeding).
  //    - The ONLY way to run hopper/feeder is through shootWithFeedCommand().
  // =========================================================================

  /**
   * Full shoot command: runs shooter, hopper, and feeder all at the same time.
   * This is the ONLY command that runs the hopper and feeder — they are never
   * started independently because balls would jam at a stationary shooter wheel.
   *
   * When toggled off, all three stop together automatically.
   */
  private Command shootWithFeedCommand() {
    return shooter.aimAtHubContinuous(drivebase)
                  .alongWith(
                      hopper.runMotorCommand(HOPPER_SPEED),
                      feeder.runMotorCommand(FEEDER_SPEED)
                  )
                  .withName("ShootWithFeed");
  }

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

public void updateVision() {
    vision.updateVision();
}
  private void registerNamedCommands()
  {
    // Turret
    NamedCommands.registerCommand("SetTurretAngle_0",   turret.setAngleCommand(0.0));
    NamedCommands.registerCommand("SetTurretAngle_90",  turret.setAngleCommand(90.0));
    NamedCommands.registerCommand("SetTurretAngle_180", turret.setAngleCommand(180.0));
    NamedCommands.registerCommand("SetTurretAngle_-90", turret.setAngleCommand(-90.0));
    NamedCommands.registerCommand("TurretAimForward",   turret.moveToAngleCommand(0.0).withTimeout(3.0));
    NamedCommands.registerCommand("TurretAimHub",       turret.aimAtTargetCommand(getHubTarget()).withTimeout(3.0));

    // Intake
    NamedCommands.registerCommand("runIntake",   intake.runMotorCommand(0.5));
    NamedCommands.registerCommand("stopIntake",  intake.stopMotorCommand());

    // Shooter spin-up without feeding (e.g. pre-spin while still path-following)
    NamedCommands.registerCommand("ShooterSpinUp", shooter.aimAtHubContinuous(drivebase));
    NamedCommands.registerCommand("ShooterStop",   Commands.runOnce(() -> shooter.set(0), shooter));

    // Full shoot: shooter + hopper + feeder.
    // This is the ONLY named command that runs the hopper and feeder.
    // Never register a standalone "runHopper" or "runFeeder" — use this instead.
    NamedCommands.registerCommand("Shoot", shootWithFeedCommand());
  }


  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle              = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity         = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity          = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    // Turret default: manual right-stick control
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
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      joystickL.button(5).onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)))));
      joystickL.button(3).onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(15.511, 6.537, new Rotation2d()))));

      // ── Operator: Turret bindings ─────────────────────────────────────────
     // operatorXbox.a().onTrue(turret.setAngleCommand(0.0));
      operatorXbox.a().whileTrue(intake.runMotorCommand(0.5));
      //operatorXbox.b().onTrue(turret.setAngleCommand(90.0));
     // operatorXbox.x().onTrue(turret.setAngleCommand(-90.0));
     // operatorXbox.y().onTrue(turret.setAngleCommand(180.0));

      // Left bumper: hard stop turret
      operatorXbox.leftBumper().onTrue(turret.stopCommand());

      // Right bumper TOGGLE: hub-lock the turret
      operatorXbox.rightBumper().toggleOnTrue(
          turret.aimAtTargetCommand(getHubTarget())
                .withName("TurretHubLock"));

      // ── Operator: Shoot binding ───────────────────────────────────────────
      //
      // Left trigger (>50%) TOGGLE — full shoot mode.
      //
      // Runs shooter + hopper + feeder simultaneously.
      // This is the ONLY way to run the hopper and feeder — they are never
      // triggered independently because a ball would jam against a stopped
      // shooter wheel. When toggled off, all three stop together.
      operatorXbox.leftTrigger(0.5).toggleOnTrue(shootWithFeedCommand());

      // ── Operator: Control bindings ───────────────────────────────────────────
      //operatorXbox.button(13).whileTrue(runIntake); //to be continued... 
      //operatorXbox.button(14).whileTrue(hopper.runMotorCommand(-HOPPER_SPEED)); change to indexer later
      //operatorXbox.button(15).whileTrue(reverseIntake);
     // operatorXbox.leftBumper().whileTrue(intake.runMotorCommand(0.4));
     // operatorXbox.
/*
Make controls for the operator using these (from the Robot X-Box Controller Buttons in the 2026 folder) 
 * intake in - left bumper
 * intake out - down arrow
 * shoot 1 fuel - right bumper
 * reverse hopper - left arrow
 * reverse indexer - right arrow 
 * vision aiming - right trigger
 * turret right - b
 * turret left - x
 * climbing up - y
 * climber down - a

 */
    }
  }


  /**
   * Returns the correct hub Translation2d for the current alliance.
   * Falls back to blue if the Driver Station hasn't reported an alliance yet.
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
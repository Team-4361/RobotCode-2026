package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.util.ShooterCalculator;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

<<<<<<< HEAD
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
=======
public class ShooterSubsystem extends SubsystemBase
{
    private final TalonFX shooterKraken;
    private final SparkFlex indexVortex;
    //private MotionMagicVelocityVoltage talonController;
        public ShooterSubsystem()
        {
            shooterKraken = new TalonFX(0); //add to constants  
            shooterKraken.getConfigurator().apply(new TalonFXConfiguration());
            var currentLimitsConfigs =   new CurrentLimitsConfigs();
            shooterKraken.setNeutralMode(NeutralModeValue.Coast);
            currentLimitsConfigs.StatorCurrentLimit = Constants.ShooterConstants.StatorCurrentLimit; // add value to constants 
            currentLimitsConfigs.StatorCurrentLimitEnable = true;
            shooterKraken.getConfigurator().refresh(currentLimitsConfigs);
            shooterKraken.getConfigurator().apply(currentLimitsConfigs);
            //talonController = new MotionMagicVelocityVoltage(0);
        indexVortex = new SparkFlex(0, MotorType.kBrushless); 
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        indexVortex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
>>>>>>> 61ad26ec607095e5744446c415d5b9d1386855db

/**
 * Shooter subsystem with full ballistic hub-targeting support.
 *
 * <p>Key features:
 * <ul>
 *   <li>Uses {@link ShooterCalculator} to compute the exact RPM needed to score
 *       from any position on the field.</li>
 *   <li>Compensates for robot velocity — the ball inherits the robot's
 *       field-relative velocity at launch, so the flywheel only needs to
 *       provide the additional speed required.</li>
 *   <li>Models aerodynamic drag and Magnus lift from backspin.</li>
 *   <li>Hub position is derived from the swerve drive's {@link Pose2d}.</li>
 * </ul>
 *
 * <p>Typical usage in RobotContainer:
 * <pre>
 *   // Spin up and keep targeting while button held
 *   new JoystickButton(driver, XboxController.Button.kRightBumper.value)
 *       .whileTrue(shooter.aimAtHubContinuous(swerve));
 *
 *   // After shooter is at speed, run the indexer to feed a ball
 *   new JoystickButton(driver, XboxController.Button.kA.value)
 *       .whileTrue(shooter.aimAtHubContinuous(swerve)
 *           .alongWith(indexer.feedCommand()
 *               .onlyWhile(() -> shooter.isAtTargetSpeed(75))));
 * </pre>
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── Motor / flywheel configuration (unchanged) ────────────────────────────

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(50, 0, 0,
                    DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(50, 0, 0,
                    DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    private final TalonFXS           krakenMotor   = new TalonFXS(4);
    private final SmartMotorController motor        =
            new TalonFXSWrapper(krakenMotor, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(6000))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    // ── Last commanded target RPM (for isAtTargetSpeed) ──────────────────────

    private double targetRPM = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────

    public ShooterSubsystem() {}

    // ═════════════════════════════════════════════════════════════════════════
    //  Hub-targeting commands  (PRIMARY API)
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Continuously recalculate and set the flywheel RPM to score from the
     * robot's current position, accounting for robot velocity.
     *
     * <p>This command never finishes on its own — use {@code .whileTrue()} or
     * combine with a button binding.
     *
     * <p>The alliance colour is automatically read from the Driver Station.
     *
     * @param swerve The swerve drive subsystem (provides pose and field velocity).
     * @return A Command that updates the flywheel speed every loop iteration.
     */
    public Command aimAtHubContinuous(SwerveSubsystem swerve) {
        return run(() -> {
            Pose2d        pose       = swerve.getPose();
            ChassisSpeeds fieldSpeeds = swerve.getFieldVelocity();
            boolean       redAlliance = isRedAlliance();

            AngularVelocity rpm = ShooterCalculator.getRPM(pose, fieldSpeeds, redAlliance);
            targetRPM = rpm.in(RPM);
            shooter.setSpeed(rpm);
        });
    }

    /**
     * Set the flywheel to the correct RPM for the robot's current pose,
     * assuming the robot is stationary.  Useful for spinning up before moving
     * into a shooting position.
     *
     * @param swerve The swerve drive subsystem (provides pose only).
     * @return A Command that sets the flywheel speed once per loop.
     */
    public Command aimAtHubStatic(SwerveSubsystem swerve) {
        return run(() -> {
            Pose2d          pose       = swerve.getPose();
            boolean         redAlliance = isRedAlliance();
            AngularVelocity rpm        = ShooterCalculator.getRPM(pose, redAlliance);
            targetRPM = rpm.in(RPM);
            shooter.setSpeed(rpm);
        });
    }

    /**
     * Snapshot command — calculates the RPM once from the current pose and
     * holds it constant.  Less accurate for moving shots but simpler to
     * sequence.
     *
     * @param swerve The swerve drive subsystem.
     * @return A Command that computes RPM once then maintains that speed.
     */
    public Command aimAtHubSnapshot(SwerveSubsystem swerve) {
        // runOnce captures the RPM at the instant the command is scheduled
        return runOnce(() -> {
            Pose2d          pose        = swerve.getPose();
            ChassisSpeeds   fieldSpeeds = swerve.getFieldVelocity();
            boolean         red         = isRedAlliance();
            AngularVelocity rpm         = ShooterCalculator.getRPM(pose, fieldSpeeds, red);
            targetRPM = rpm.in(RPM);
            shooter.setSpeed(rpm);
        });
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Basic controls (unchanged from original)
    // ═════════════════════════════════════════════════════════════════════════

    /** @return Current measured flywheel angular velocity. */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Directly set flywheel to a specific speed.
     *
     * @param speed Desired angular velocity.
     * @return Command that holds that speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    /**
     * Open-loop duty-cycle control.
     *
     * @param dutyCycle Output fraction (−1.0 to +1.0).
     * @return Command that applies the duty cycle.
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Ready-to-shoot check
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Returns {@code true} when the flywheel has spun up to within
     * {@code toleranceRPM} of the last commanded target speed.
     *
     * <p>Use this as a gate before feeding a ball:
     * <pre>
     *   .onlyWhile(() -> shooter.isAtTargetSpeed(75))
     * </pre>
     *
     * @param toleranceRPM Acceptable RPM error.  Typical value: 50–100 RPM.
     * @return {@code true} if the flywheel is at speed.
     */
    public boolean isAtTargetSpeed(double toleranceRPM) {
        return Math.abs(getVelocity().in(RPM) - targetRPM) <= toleranceRPM;
    }

    /**
     * Returns the RPM the shooter is currently targeting.
     * Useful for SmartDashboard display or trigger conditions.
     *
     * @return Target RPM.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Periodic
    // ═════════════════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Helpers
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Reads the current alliance from the Driver Station.
     * Defaults to {@code false} (blue) if the alliance is unavailable.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
<<<<<<< HEAD
=======
    public void stopShooter()
   {
        shooterKraken.stopMotor();
        
   }
   public void stopIndexer()
   {
        indexVortex.stopMotor();
   }
   public void stopMotors()
   {
        shooterKraken.stopMotor();
        indexVortex.stopMotor();
   }
   public double getRPM()
   {
         return shooterKraken.getVelocity().getValueAsDouble();
   }
    
>>>>>>> 61ad26ec607095e5744446c415d5b9d1386855db
}
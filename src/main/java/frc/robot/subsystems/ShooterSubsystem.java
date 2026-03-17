package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import frc.robot.util.ShooterCalculator;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import edu.wpi.first.units.Units;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

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

SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
.withClosedLoopController(
    0.00016541, 0, 0,
    RPM.of(5000),
    RotationsPerSecondPerSecond.of(2500)   // ← fix units here too
)
.withSimClosedLoopController(
    0.00016541, 0, 0,
    RPM.of(5000),
    RotationsPerSecondPerSecond.of(2500)
)
.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
.withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))

  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));


private final TalonFX krakenMotor = new TalonFX(10);
    private final SmartMotorController motor        =
        new TalonFXWrapper(krakenMotor, DCMotor.getKrakenX60(1), smcConfig);

        
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(3.375)) //flywheel mass (6 flywheels)
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

    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    // ═════════════════════════════════════════════════════════════════════════
    //  System identification routines
    // ═════════════════════════════════════════════════════════════════════════
        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),  // reduce to prevent brownout
                null,
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> krakenMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,  // ← null because SignalLogger handles logging automatically
                this
            )
        );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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
        SmartDashboard.putNumber("Shooter RPM", shooter.getSpeed().in(RPM));
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
}
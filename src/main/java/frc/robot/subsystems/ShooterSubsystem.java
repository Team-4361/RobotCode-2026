package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Shooter subsystem — one long flywheel driven by two Krakens.
 *
 * <p>Both motors drive the same mechanism, so the follower Kraken is wired
 * in via {@code withLooselyCoupledFollowers()}.  The primary motor's encoder
 * is used for all closed-loop feedback; the follower mirrors every velocity /
 * position setpoint automatically.
 *
 * <p>SysId applies voltage to both Krakens simultaneously so the
 * characterisation reflects the real loaded system.  After running SysId:
 * <ol>
 *   <li>Open the CTRE log in SysId Analyzer → Simple Motor.</li>
 *   <li>Paste the new kS / kV / kA into the {@code SimpleMotorFeedforward} below.</li>
 *   <li>Paste the recommended kP into {@code withClosedLoopController}.</li>
 * </ol>
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── CAN IDs ───────────────────────────────────────────────────────────────
    private static final int PRIMARY_CAN_ID  = 10;
    private static final int FOLLOWER_CAN_ID = 11;

    // =========================================================================
    //  Motor hardware
    // =========================================================================

    private final TalonFX primaryKraken  = new TalonFX(PRIMARY_CAN_ID);
    private final TalonFX followerKraken = new TalonFX(FOLLOWER_CAN_ID);

    // =========================================================================
    //  Follower SmartMotorController
    //  (configured first so it can be passed into the primary's config)
    // =========================================================================

    // The follower needs its own minimal config so YAMS can wrap it.
    // It won't run its own closed loop — the primary drives it via
    // withLooselyCoupledFollowers().
    private final SmartMotorControllerConfig followerSmcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("FollowerShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
            // Invert the follower if it's physically mounted in the opposite
            // direction from the primary on the same shaft/belt.
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController followerMotor =
        new TalonFXWrapper(followerKraken, DCMotor.getKrakenX60(1), followerSmcConfig);

    // =========================================================================
    //  Primary SmartMotorController  (owns the closed loop + follower)
    // =========================================================================

    private final SmartMotorControllerConfig primarySmcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                0.00016541, 0, 0,
                RPM.of(5000),
                RotationsPerSecondPerSecond.of(2500))
            .withSimClosedLoopController(
                0.00016541, 0, 0,
                RPM.of(5000),
                RotationsPerSecondPerSecond.of(2500))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withTelemetry("PrimaryShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            // Follower mirrors every velocity/position setpoint from the primary.
            // Note: this does NOT forward DutyCycle or Voltage requests — those
            // are handled manually in SysId below.
            .withLooselyCoupledFollowers(followerMotor);



    private final SmartMotorController primaryMotor =
        new TalonFXWrapper(primaryKraken, DCMotor.getKrakenX60(1), primarySmcConfig);

    // =========================================================================
    //  FlyWheel mechanism  (single, driven by the primary)
    // =========================================================================

    private final FlyWheelConfig shooterConfig =
        new FlyWheelConfig(primaryMotor)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(8))          // total flywheel mass (all 6 flywheels)
            .withUpperSoftLimit(RPM.of(6000))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    // =========================================================================
    //  State
    // =========================================================================

    private double targetRPM = 0.0;

    // =========================================================================
    //  SysId
    //  Both Krakens are driven at the same voltage so the characterisation
    //  captures the full mechanical load.  The follower voltage is negated
    //  because withMotorInverted(true) is set on it above — keep them in sync.
    // =========================================================================

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("ShooterSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
                // Primary runs forward.
                primaryKraken.setControl(m_voltReq.withOutput(volts.in(Volts)));
                // Follower is inverted, so negate to keep both pushing the same way.
                // If you set withMotorInverted(false) on the follower above, remove the minus.
                followerKraken.setControl(m_voltReq.withOutput(-volts.in(Volts)));
            },
            null, // SignalLogger handles logging via CTRE automatically
            this));

    // =========================================================================
    //  Constructor
    // =========================================================================

    public ShooterSubsystem() {}

    // =========================================================================
    //  Public command API
    // =========================================================================

    /** @return Current measured flywheel speed. */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Spin the flywheel to a specific speed.
     * The follower tracks automatically via the loosely coupled config.
     *
     * @param speed Desired angular velocity.
     * @return Command that holds that speed until interrupted.
     */
    public Command setVelocity(AngularVelocity speed) {
        targetRPM = speed.in(RPM);
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

    // =========================================================================
    //  SysId commands — bind these to controller buttons in RobotContainer
    // =========================================================================

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // =========================================================================
    //  Ready-to-shoot check
    // =========================================================================

    /**
     * Returns {@code true} when the flywheel is within {@code toleranceRPM}
     * of the last commanded target speed.
     *
     * <pre>
     *   indexer.feedCommand().onlyWhile(() -> shooter.isAtTargetSpeed(75))
     * </pre>
     *
     * @param toleranceRPM Acceptable RPM error (typical: 50–100 RPM).
     */
    public boolean isAtTargetSpeed(double toleranceRPM) {
        return Math.abs(getVelocity().in(RPM) - targetRPM) <= toleranceRPM;
    }

    /** Returns the RPM currently being targeted. */
    public double getTargetRPM() {
        return targetRPM;
    }

    // =========================================================================
    //  Periodic
    // =========================================================================

    @Override
    public void periodic() {
        shooter.updateTelemetry();
        SmartDashboard.putNumber("Shooter/RPM",        getVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putBoolean("Shooter/At Speed",  isAtTargetSpeed(75));
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    // =========================================================================
    //  Helpers
    // =========================================================================

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
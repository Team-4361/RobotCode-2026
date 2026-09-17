package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.TunableNumber;

/**
 * Shooter subsystem — two Krakens on one flywheel, native Phoenix 6, no YAMS.
 *
 * FOLLOWER CONTROL (fixed): the follower Kraken now uses CTRE's native
 * {@link Follower} control request instead of running its own independent
 * VelocityVoltage closed loop off its own encoder. Two independent velocity
 * loops on the same physical shaft can fight each other on any encoder
 * mismatch, belt slip, or backlash — each one trying to converge its own
 * shaft reading to the same RPM independently, rather than one motor simply
 * mirroring the other's output like it should. The Follower request is a
 * single field, built ONCE in the constructor and applied there — matching
 * the "linked at construction" principle: if you only wire the follower
 * during specific control-mode calls (e.g. only in setTargetRPM), it goes
 * uncontrolled during any other call path (legacy setSPEED, SysId, etc).
 * Building the request in the constructor means the follower automatically
 * mirrors the primary across EVERY control mode without touching it again.
 *
 * FOLLOWER_INVERTED still controls whether the second Kraken needs to spin
 * opposite the primary (mounted mechanically opposite on the same
 * shaft/belt) — this maps directly onto Follower's opposeMasterDirection
 * parameter, so flip that one flag if it's mounted backwards.
 *
 * BACKWARDS COMPATIBLE: setSPEED, set, revShooter, STOPP, stop, speeeed,
 * getVelocity, isAtTargetSpeed, getTargetRPM, sysId commands all keep their
 * original names/behavior. RobotContainer requires zero changes.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── CAN IDs (unchanged) ─────────────────────────────────────────────────
    private static final int PRIMARY_CAN_ID  = 13;
    private static final int FOLLOWER_CAN_ID = 14;

    /** Rotations of motor per rotation of flywheel. Update if geared. */
    private static final double GEAR_RATIO = 1.0;

    /**
     * Set true if the follower Kraken is mounted physically opposite the
     * primary (needs to spin the other way to push the flywheel the same
     * direction). This is CTRE's Follower.opposeMasterDirection — flip this
     * single flag, do NOT touch motor-invert config for this.
     */
    private static final boolean FOLLOWER_INVERTED = false;

    // ── Power management ─────────────────────────────────────────────────
    private static final double STATOR_CURRENT_LIMIT_A       = 80.0;
    private static final double SUPPLY_CURRENT_LIMIT_A       = 50.0;
    private static final double SUPPLY_CURRENT_LOWER_LIMIT_A = 40.0;
    private static final double SUPPLY_CURRENT_LOWER_TIME_S  = 1.0;

    private final TalonFX primaryKraken  = new TalonFX(PRIMARY_CAN_ID);
    private final TalonFX followerKraken = new TalonFX(FOLLOWER_CAN_ID);

    // Only the PRIMARY ever gets a closed-loop / voltage / duty-cycle request
    // built and sent — the follower request below is applied ONCE and then
    // the follower firmware handles mirroring on its own from then on.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut      m_voltReq       = new VoltageOut(0.0);

    /**
     * Built once, applied once in the constructor. After that, the follower
     * Kraken mirrors whatever control request is sent to the primary —
     * VelocityVoltage, VoltageOut, DutyCycleOut, all of it — entirely at the
     * firmware/CAN level. This field only exists so we have a single named
     * place documenting the relationship; it is never re-applied per-call.
     */
    private final Follower followerRequest = new Follower(
        PRIMARY_CAN_ID,
        FOLLOWER_INVERTED
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned
    );
    private double targetRPM = 0.0;

    // ── Live-tunable PID/FF gains for the closed-loop RPM path ─────────────
    private final TunableNumber tKP = new TunableNumber("Shooter/Tuning/kP", 0.11);
    private final TunableNumber tKI = new TunableNumber("Shooter/Tuning/kI", 0.0);
    private final TunableNumber tKD = new TunableNumber("Shooter/Tuning/kD", 0.0);
    private final TunableNumber tKS = new TunableNumber("Shooter/Tuning/kS", 0.27937);
    private final TunableNumber tKV = new TunableNumber("Shooter/Tuning/kV", 0.089836);
    private final TunableNumber tKA = new TunableNumber("Shooter/Tuning/kA", 0.014557);

    // SysId only ever talks to the primary — the follower request already
    // applied in the constructor takes care of mirroring the voltage step.
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("ShooterSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> primaryKraken.setControl(m_voltReq.withOutput(volts.in(Volts))),
            null,
            this));

    public ShooterSubsystem() {
        TalonFXConfiguration primaryConfig = new TalonFXConfiguration();
        primaryConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT_A)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME_S);

        primaryConfig.Slot0 = new Slot0Configs()
            .withKP(tKP.get()).withKI(tKI.get()).withKD(tKD.get())
            .withKS(tKS.get()).withKV(tKV.get()).withKA(tKA.get());

        primaryConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
        primaryConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        primaryKraken.getConfigurator().apply(primaryConfig);

        // The follower still gets its own current-limit config — current
        // limits are enforced per-device by firmware regardless of which
        // motor is "leading" the control loop, so this isn't redundant with
        // the Follower request above (which only governs output, not limits).
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits = primaryConfig.CurrentLimits;
        followerKraken.getConfigurator().apply(followerConfig);

        // Apply the Follower request ONCE, here in the constructor, so the
        // follower mirrors the primary across every control mode this
        // subsystem ever uses — legacy duty-cycle, new velocity control,
        // and SysId voltage steps alike — without needing to be touched
        // again at any other call site.
        followerKraken.setControl(followerRequest);
    }

    // =========================================================================
    //  EXISTING API — unchanged signatures, unchanged behavior
    // =========================================================================

    public AngularVelocity getVelocity() {
        return primaryKraken.getVelocity().getValue();
    }

    public Command setVelocity(AngularVelocity speed) {
        targetRPM = speed.in(RPM);
        return this.run(() -> setTargetRPM(targetRPM));
    }

    /** Legacy open-loop duty-cycle drive — commands only the primary; the follower mirrors it automatically. */
    public void changeShooterSpeed(double speed) {
        primaryKraken.set(speed);
    }

    public void stopShooterSpeed() {
        primaryKraken.set(0);
    }

    public Command setSPEED(double speed) {
        return this.run(() -> changeShooterSpeed(speed));
    }

    public Command revShooter(double speed) {
        return this.runOnce(() -> changeShooterSpeed(speed));
    }

    public Command STOPP() {
        return this.run(() -> stopShooterSpeed());
    }

    public Command set(double dutyCycle) {
        return this.run(() -> changeShooterSpeed(dutyCycle));
    }

    public Command speeeed(double dutyCycle) {
        return this.run(() -> changeShooterSpeed(dutyCycle));
    }

    public Command stop() {
        return this.runOnce(() -> stopShooterSpeed());
    }

    public boolean isAtTargetSpeed(double toleranceRPM) {
        return Math.abs(getVelocity().in(RPM) - targetRPM) <= toleranceRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // =========================================================================
    //  NEW — additive closed-loop RPM control. Commands only the primary;
    //  the follower request applied in the constructor mirrors it.
    // =========================================================================

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        double motorRotPerSec = (rpm / 60.0) * GEAR_RATIO;
        primaryKraken.setControl(velocityRequest.withVelocity(motorRotPerSec));
    }

    public Command setRPMCommand(double rpm) {
        return this.run(() -> setTargetRPM(rpm));
    }

    public Command revShooterRPM(double rpm) {
        return this.runOnce(() -> setTargetRPM(rpm));
    }

    // =========================================================================
    //  Periodic
    // =========================================================================

    @Override
    public void periodic() {
        boolean changed = tKP.poll(v -> {}) | tKI.poll(v -> {}) | tKD.poll(v -> {})
                         | tKS.poll(v -> {}) | tKV.poll(v -> {}) | tKA.poll(v -> {});
        if (changed) {
            Slot0Configs newSlot0 = new Slot0Configs()
                .withKP(tKP.get()).withKI(tKI.get()).withKD(tKD.get())
                .withKS(tKS.get()).withKV(tKV.get()).withKA(tKA.get());
            // Only the primary runs closed-loop, so only it needs new gains.
            primaryKraken.getConfigurator().apply(newSlot0);
        }

        SmartDashboard.putNumber("Shooter/RPM",        getVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter/Follower RPM", followerKraken.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putBoolean("Shooter/At Speed",  isAtTargetSpeed(75));
        SmartDashboard.putNumber("Shooter/Supply Current (A)", primaryKraken.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Stator Current (A)", primaryKraken.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bus Voltage (V)",    primaryKraken.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Follower Current (A)", followerKraken.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Shooter/RPM",           getVelocity().in(RPM));
        Logger.recordOutput("Shooter/FollowerRPM",   followerKraken.getVelocity().getValue().in(RPM));
        Logger.recordOutput("Shooter/TargetRPM",     targetRPM);
        Logger.recordOutput("Shooter/AtTargetSpeed", isAtTargetSpeed(75));
        Logger.recordOutput("Shooter/SupplyCurrentA", primaryKraken.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/StatorCurrentA", primaryKraken.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/BusVoltageV",    primaryKraken.getSupplyVoltage().getValueAsDouble());

    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
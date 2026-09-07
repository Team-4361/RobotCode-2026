package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
 * NO Follower CLASS: instead of linking the follower Kraken via CTRE's
 * Follower control request (which was causing IDE/classpath resolution
 * issues), both motors are commanded directly and explicitly every time,
 * in every method. This is functionally equivalent — just less "magic" —
 * and has the side benefit that both motors show independent, explicit
 * commands in Phoenix Tuner X rather than one silently mirroring the other.
 *
 * FOLLOWER_INVERTED controls whether the second Kraken needs the opposite
 * sign to spin the correct physical direction (set true if it's mounted
 * mechanically opposite the primary on the same shaft/belt).
 *
 * BACKWARDS COMPATIBLE: setSPEED, set, revShooter, STOPP, stop, speeeed,
 * getVelocity, isAtTargetSpeed, getTargetRPM, sysId commands all keep their
 * original names/behavior. RobotContainer requires zero changes.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── CAN IDs (unchanged) ─────────────────────────────────────────────────
    private static final int PRIMARY_CAN_ID  = 13;
    private static final int FOLLOWER_CAN_ID = 14;
    private static final DutyCycleOut zero = new DutyCycleOut(0);

    /** Rotations of motor per rotation of flywheel. Update if geared. */
    private static final double GEAR_RATIO = 1.0;

    /**
     * Set true if the follower Kraken is mounted physically opposite the
     * primary (needs to spin the other way to push the flywheel the same
     * direction). Flip this — do NOT use motor-invert config for this,
     * keep it isolated to this one flag so it's easy to find and correct.
     */
    private static final boolean FOLLOWER_INVERTED = false;
    private static final double FOLLOWER_SIGN = FOLLOWER_INVERTED ? -1.0 : 1.0;

    // ── Power management ─────────────────────────────────────────────────
    private static final double STATOR_CURRENT_LIMIT_A       = 80.0;
    private static final double SUPPLY_CURRENT_LIMIT_A       = 50.0;
    private static final double SUPPLY_CURRENT_LOWER_LIMIT_A = 40.0;
    private static final double SUPPLY_CURRENT_LOWER_TIME_S  = 1.0;

    private final TalonFX primaryKraken  = new TalonFX(PRIMARY_CAN_ID);
    private final TalonFX followerKraken = new TalonFX(FOLLOWER_CAN_ID);

    private final VelocityVoltage velocityRequestPrimary  = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityRequestFollower = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut      m_voltReq               = new VoltageOut(0.0);

    private double targetRPM = 0.0;

    // ── Live-tunable PID/FF gains for the closed-loop RPM path ─────────────
    private final TunableNumber tKP = new TunableNumber("Shooter/Tuning/kP", 0.11);
    private final TunableNumber tKI = new TunableNumber("Shooter/Tuning/kI", 0.0);
    private final TunableNumber tKD = new TunableNumber("Shooter/Tuning/kD", 0.0);
    private final TunableNumber tKS = new TunableNumber("Shooter/Tuning/kS", 0.27937);
    private final TunableNumber tKV = new TunableNumber("Shooter/Tuning/kV", 0.089836);
    private final TunableNumber tKA = new TunableNumber("Shooter/Tuning/kA", 0.014557);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("ShooterSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
                primaryKraken.setControl(m_voltReq.withOutput(volts.in(Volts)));
                followerKraken.setControl(m_voltReq.withOutput(FOLLOWER_SIGN * volts.in(Volts)));
            },
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

        // Follower gets the SAME PID/FF gains and current limits as the
        // primary — it runs its own independent closed loop on its own
        // encoder rather than blindly mirroring, so if it's mechanically
        // sound it converges to the same RPM as the primary.
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits = primaryConfig.CurrentLimits;
        followerConfig.Slot0 = primaryConfig.Slot0;
        followerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        followerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
        followerKraken.getConfigurator().apply(followerConfig);
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

    /** Legacy open-loop duty-cycle drive — now commands both motors explicitly. */
    public void changeShooterSpeed(double speed) {
        primaryKraken.setControl(new DutyCycleOut(speed));
        followerKraken.setControl(new DutyCycleOut(FOLLOWER_SIGN * speed));
    }

    public void stopShooterSpeed() {
        primaryKraken.setControl(new DutyCycleOut(0));
        followerKraken.setControl(new DutyCycleOut(0));
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
    //  NEW — additive closed-loop RPM control. Commands BOTH motors directly.
    // =========================================================================

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        double motorRotPerSec = (rpm / 60.0) * GEAR_RATIO;
        primaryKraken.setControl(velocityRequestPrimary.withVelocity(motorRotPerSec));
        followerKraken.setControl(velocityRequestFollower.withVelocity(FOLLOWER_SIGN * motorRotPerSec));
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
            primaryKraken.getConfigurator().apply(newSlot0);
            followerKraken.getConfigurator().apply(newSlot0);
        }

        SmartDashboard.putNumber("Shooter/RPM",        getVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter/Follower RPM", followerKraken.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putBoolean("Shooter/At Speed",  isAtTargetSpeed(75));
        SmartDashboard.putNumber("Shooter/Supply Current (A)", primaryKraken.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Stator Current (A)", primaryKraken.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bus Voltage (V)",    primaryKraken.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Follower Current (A)", followerKraken.getSupplyCurrent().getValueAsDouble());
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
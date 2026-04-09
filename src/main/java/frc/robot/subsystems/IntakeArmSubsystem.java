package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {

    // --- Constants ---
    private static final int    LEADER_ID          = 2;
    private static final int    FOLLOWER_ID        = 3;
    private static final double DEPLOYED_POSITION  = 22.5;  // rotations — tune this
    private static final double STOWED_POSITION    = 0.0;
    private static final double kP                 = 0.1;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.01;
    private static final double POSITION_TOLERANCE = 0.5;   // rotations

    // --- Hardware ---
    private final SparkMax        leader;
    private final SparkMax        follower;
    private final RelativeEncoder encoder;
    private final PIDController   pid;

    public IntakeArmSubsystem() {
        leader   = new SparkMax(LEADER_ID,   MotorType.kBrushless);
        follower = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);

        // Configure leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure follower — mirrors leader, same direction
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .follow(leader, false); // flip to true if it spins the wrong way
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();
        encoder.setPosition(0.0);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE);
        pid.setSetpoint(STOWED_POSITION);
    }

    // --- Getters ---

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    // --- Internal Control ---

    public void runPID() {
        double output = pid.calculate(getPosition());
        output = Math.max(-1.0, Math.min(1.0, output));
        leader.set(output); // follower mirrors automatically
    }

    public void stop() {
        leader.stopMotor(); // follower mirrors automatically
        pid.reset();
    }

    public void rezero() {
        encoder.setPosition(0.0);
        pid.setSetpoint(STOWED_POSITION);
        pid.reset();
    }

    // --- Commands ---

    /** Deploys the intake arm down and holds position. */
    public Command deployCommand() {
        return this.run(() -> {
            pid.setSetpoint(DEPLOYED_POSITION);
            runPID();
        }).until(this::atSetpoint);
    }

    /** Stows the intake arm up and holds position. */
    public Command stowCommand() {
        return this.run(() -> {
            pid.setSetpoint(STOWED_POSITION);
            runPID();
        }).until(this::atSetpoint);
    }

    /** Re-zeros encoder at current position — use when arm is physically stowed. */
    public Command rezeroCommand() {
        return this.runOnce(this::rezero);
    }

    /** Manual override — hold button to nudge arm, stops on release. */
    public Command manualCommand(double speed) {
        return this.startEnd(
            () -> leader.set(speed), // follower mirrors automatically
            this::stop
        );
    }

    // --- Periodic ---

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeArm/Position (rot)", getPosition());
        SmartDashboard.putNumber("IntakeArm/Setpoint",       pid.getSetpoint());
        SmartDashboard.putBoolean("IntakeArm/AtGoal",        pid.atSetpoint());
        SmartDashboard.putNumber("IntakeArm/Leader Current",   leader.getOutputCurrent());
        SmartDashboard.putNumber("IntakeArm/Follower Current", follower.getOutputCurrent());
    }
}
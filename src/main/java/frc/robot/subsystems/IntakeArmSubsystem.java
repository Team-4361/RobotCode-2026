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
    private static final int    LEADER_ID          = 10;
    private static final int    FOLLOWER_ID        = 11;
    private static final double DEPLOYED_POSITION  = 17.714244842529297;  // rotations — tune this
    private static final double STOWED_POSITION    = 0.0;
    private static final double STOWED_POSITION_PARTIAL  = 6.142855167388916;

    private static final double kP                 = 0.1;
    private static final double kI                 = 0.0;
    private static final double kD                 = 0.01;
    private static final double POSITION_TOLERANCE = 0.5;   // rotations
    private final Command deployAutoCmd;
    private boolean isDeploying = false;
    // --- Hardware ---
    private final SparkMax        leader;
    private final SparkMax        follower;
    private final RelativeEncoder encoder;
    private final PIDController   pid;

    public IntakeArmSubsystem() {
        leader   = new SparkMax(LEADER_ID,   MotorType.kBrushless);
        follower = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE);
        pid.setSetpoint(STOWED_POSITION);

        deployAutoCmd = this.run(() -> {
            pid.setSetpoint(DEPLOYED_POSITION);
            runPID();
        }).until(this::atSetpoint);

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
        output = Math.max(-0.3, Math.min(0.3, output));
        leader.set(output); 
        follower.set(-output);
    }

    public void stop() {
        leader.stopMotor(); 
        follower.stopMotor();
        pid.reset();
    }
    public void runstuff() {
        leader.set(0.2); 
        follower.set(-0.2);
    }
        public void runstuffdown() {
        leader.set(-0.1); 
        follower.set(0.1);
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

public Command deployCommandAuto() {
    return this.runOnce(() -> pid.setSetpoint(DEPLOYED_POSITION))
        .andThen(this.run(this::runPID)
            .until(this::atSetpoint));
}
    

    /** Stows the intake arm up and holds position. */
    public Command stowCommand() {
        return this.run(() -> {
            pid.setSetpoint(STOWED_POSITION);
            runPID();
        }).until(this::atSetpoint);
    }


        public Command stowHalfCommand() {
        return this.run(() -> {
            pid.setSetpoint(STOWED_POSITION_PARTIAL);
            runPID();
        }).until(this::atSetpoint);
    }

            public Command deploySimple() {
        return this.runOnce(() -> {
            runstuff();
        }).withTimeout(0.5);
    }
    /** Re-zeros encoder at current position — use when arm is physically stowed. */
    public Command rezeroCommand() {
        return this.runOnce(this::rezero);
    }



        public Command manualCommand() {
        return this.startEnd(this::runstuff, this::stop);
    }
        public Command manualCommandDown() {
        return this.startEnd(this::runstuffdown, this::stop);
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeArm/Position (rot)", getPosition());
        SmartDashboard.putNumber("IntakeArm/Setpoint",       pid.getSetpoint());
        SmartDashboard.putBoolean("IntakeArm/AtGoal",        pid.atSetpoint());
        SmartDashboard.putNumber("IntakeArm/Leader Current",   leader.getOutputCurrent());
        SmartDashboard.putNumber("IntakeArm/Follower Current", follower.getOutputCurrent());

        if(atSetpoint()) {
            stop();
            }


    }
}
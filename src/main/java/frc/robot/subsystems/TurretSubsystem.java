package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {

  // ========== FIELD-RELATIVE AIMING FIELDS ==========
  // Swerve drive reference for pose
  private SwerveSubsystem swerveSubsystem;
  
  // Target position on field (meters)
  private Translation2d targetPosition = new Translation2d(0, 0);
  
  // Current desired angle (in degrees)
  private double desiredAngleDegrees = 0.0;
  
  // Turret angle limits (in degrees, robot-relative)
  private static final double MIN_TURRET_ANGLE = -180.0;
  private static final double MAX_TURRET_ANGLE = 180.0;

  // ========== ORIGINAL YAMG CONSTANTS ==========
  // Constants
  private final DCMotor dcMotor = DCMotor.getNeo550(1);
  private final int canID = 1;
  private final double gearRatio = 10;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double kS = 0;
  private final double kV = 0;
  private final double kA = 0;
  private final double kG = 0; // Unused for pivots
  private final double maxVelocity = 1; // rad/s
  private final double maxAcceleration = 1; // rad/s²
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(
    kS, // kS
    0, // kG - Pivot doesn't need gravity compensation
    kV, // kV
    kA // kA
  );

  // Motor controller
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /**
   * Creates a new Turret Subsystem.
   */
  public TurretSubsystem() {
    this(null); // Default constructor for backwards compatibility
  }

  /**
   * Creates a new Turret Subsystem with field-relative aiming support.
   * @param swerveSubsystem The YAGSL swerve subsystem for pose information
   */
  public TurretSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    
    // Initialize motor controller
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motor = new SparkMax(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    motorConfig.smartCurrentLimit((int) statorCurrentLimit); 
    //note: I literally just typecasted this to an int. We probably need a new method for this.

    // Configure Feedback and Feedforward
    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    motorConfig.closedLoop.feedForward.kG(kG);

    // Configure Encoder Gear Ratio
    motorConfig.encoder
      .positionConversionFactor(1 / gearRatio)
      .velocityConversionFactor((1 / gearRatio) / 60); // Convert RPM to RPS

    // Save configuration
    motor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
    pivotSim = new SingleJointedArmSim(
      dcMotor, // Motor type
      gearRatio,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(-180), // Min angle (rad) - Full rotation
      Units.degreesToRadians(180), // Max angle (rad) - Full rotation
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(0) // Starting position (rad)
    );
  }

  // ========== FIELD-RELATIVE AIMING METHODS ==========
  
  /**
   * Set the target position on the field that the turret should aim at
   * @param targetPosition Field-relative position in meters (x, y)
   */
  public void setTargetPosition(Translation2d targetPosition) {
    this.targetPosition = targetPosition;
  }
  
  /**
   * Set the target position on the field using x and y coordinates
   * @param x X coordinate in meters
   * @param y Y coordinate in meters
   */
  public void setTargetPosition(double x, double y) {
    this.targetPosition = new Translation2d(x, y);
  }
  
  /**
   * Calculate the required turret angle to aim at the target position
   * This accounts for:
   * 1. Robot position on field
   * 2. Robot rotation
   * 3. Turret angle limits (-180 to 180 degrees)
   * @return Required turret angle in degrees (robot-relative, -180 to 180)
   */
  public double calculateTurretAngle() {
    if (swerveSubsystem == null) {
      System.err.println("Warning: SwerveSubsystem not set! Cannot calculate field-relative angle.");
      return 0.0;
    }
    
    // Get current robot pose from YAGSL
    Pose2d robotPose = swerveSubsystem.getPose();
    Translation2d robotPosition = robotPose.getTranslation();
    Rotation2d robotRotation = robotPose.getRotation();
    
    // Calculate vector from robot to target
    Translation2d robotToTarget = targetPosition.minus(robotPosition);
    
    // Calculate field-relative angle to target
    double fieldRelativeAngle = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
    fieldRelativeAngle = Math.toDegrees(fieldRelativeAngle);
    
    // Convert to robot-relative angle
    // Subtract robot's rotation to get the angle relative to robot's front
    double robotRelativeAngle = fieldRelativeAngle - robotRotation.getDegrees();
    
    // Normalize angle to -180 to 180 range
    robotRelativeAngle = normalizeAngle(robotRelativeAngle);
    
    // Clamp to turret limits
    robotRelativeAngle = MathUtil.clamp(robotRelativeAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    
    return robotRelativeAngle;
  }
  
  /**
   * Normalize an angle to the range -180 to 180 degrees
   * @param angle Angle in degrees
   * @return Normalized angle in degrees
   */
  private double normalizeAngle(double angle) {
    angle = angle % 360.0;
    if (angle > 180.0) {
      angle -= 360.0;
    } else if (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }
  
  /**
   * Calculate the shortest path to the target angle considering wrapping
   * @param currentAngle Current turret angle in degrees
   * @param targetAngle Target turret angle in degrees
   * @return Optimal target angle (may wrap around)
   */
  private double calculateOptimalAngle(double currentAngle, double targetAngle) {
    // Calculate the error
    double error = targetAngle - currentAngle;
    
    // Normalize error to -180 to 180
    error = normalizeAngle(error);
    
    // Calculate the optimal target considering wrapping
    double optimalTarget = currentAngle + error;
    
    // Ensure we stay within limits
    if (optimalTarget < MIN_TURRET_ANGLE) {
      optimalTarget = MIN_TURRET_ANGLE;
    } else if (optimalTarget > MAX_TURRET_ANGLE) {
      optimalTarget = MAX_TURRET_ANGLE;
    }
    
    return optimalTarget;
  }
  
  /**
   * Aim the turret at the target position automatically
   * Call this in a command's execute() method for continuous aiming
   */
  public void aimAtTarget() {
    double targetAngle = calculateTurretAngle();
    desiredAngleDegrees = targetAngle;
    setAngle(targetAngle);
  }
  
  /**
   * Check if turret is at the desired angle
   * @param tolerance Tolerance in degrees
   * @return True if within tolerance
   */
  public boolean atSetpoint(double tolerance) {
    double currentAngle = Units.radiansToDegrees(getPositionRadians());
    return Math.abs(currentAngle - desiredAngleDegrees) < tolerance;
  }
  
  /**
   * Check if turret is at the desired angle with default tolerance
   * @return True if within 2 degrees
   */
  public boolean atSetpoint() {
    return atSetpoint(2.0);
  }
  
  /**
   * Get distance to target
   * @return Distance to target in meters
   */
  public double getDistanceToTarget() {
    if (swerveSubsystem == null) {
      return 0.0;
    }
    Pose2d robotPose = swerveSubsystem.getPose();
    Translation2d robotPosition = robotPose.getTranslation();
    return robotPosition.getDistance(targetPosition);
  }

  // ========== FIELD-RELATIVE AIMING COMMANDS ==========
  
  /**
   * Creates a command to continuously aim at a target position
   * @param target Field-relative target position (x, y in meters)
   * @return A command that aims the turret at the target
   */
  public Command aimAtTargetCommand(Translation2d target) {
    return run(() -> {
      setTargetPosition(target);
      aimAtTarget();
    });
  }
  
  /**
   * Creates a command to aim at a target and wait until on target
   * @param target Field-relative target position (x, y in meters)
   * @param timeoutSeconds Maximum time to wait
   * @return A command that aims and waits
   */
  public Command aimAndWaitCommand(Translation2d target, double timeoutSeconds) {
    return aimAtTargetCommand(target)
      .until(() -> atSetpoint())
      .withTimeout(timeoutSeconds);
  }

  // ========== ORIGINAL YAMG METHODS ==========

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    // Add field-relative aiming telemetry
    if (swerveSubsystem != null) {
      SmartDashboard.putNumber("Turret/Current Angle", Units.radiansToDegrees(getPositionRadians()));
      SmartDashboard.putNumber("Turret/Desired Angle", desiredAngleDegrees);
      SmartDashboard.putNumber("Turret/Calculated Target Angle", calculateTurretAngle());
      SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
      SmartDashboard.putNumber("Turret/Distance to Target", getDistanceToTarget());
      SmartDashboard.putNumber("Turret/Target X", targetPosition.getX());
      SmartDashboard.putNumber("Turret/Target Y", targetPosition.getY());
      
      Pose2d robotPose = swerveSubsystem.getPose();
      SmartDashboard.putNumber("Turret/Robot X", robotPose.getX());
      SmartDashboard.putNumber("Turret/Robot Y", robotPose.getY());
      SmartDashboard.putNumber("Turret/Robot Rotation", robotPose.getRotation().getDegrees());
    }
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    pivotSim.setInput(getVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        pivotSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      pivotSim.getVelocityRadPerSec() * gearRatio
    ).in(RotationsPerSecond);
    motorSim.iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
  }

  /**
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return encoder.getPosition();
  }
  
  /**
   * Get the current position in Radians
   * @return Position in Radians (0 to 2π)
   */
  public double getPositionRadians() {
    // Convert rotations to radians
    return encoder.getPosition() * 2.0 * Math.PI;
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Set pivot angle.
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set pivot angle with acceleration.
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    desiredAngleDegrees = angleDegrees;
    
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    sparkPidController.setSetpoint(
      positionRotations,
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set pivot angular velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    sparkPidController.setSetpoint(
      velocityRotations,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the pivot to a specific angle.
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the pivot to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> {
      double currentAngle = Units.radiansToDegrees(getPositionRadians());
      double error = angleDegrees - currentAngle;
      double velocityDegPerSec =
        Math.signum(error) *
        Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
      setVelocity(velocityDegPerSec);
    })
      .until(() -> {
        double currentAngle = Units.radiansToDegrees(getPositionRadians());
        return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
      })
      .finallyDo(interrupted -> setVelocity(0));
  }

  /**
   * Creates a command to stop the pivot.
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the pivot at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }
}
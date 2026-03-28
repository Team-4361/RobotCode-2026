// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Rotations;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkSim;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class TurretSubsystem extends SubsystemBase {
//   private SwerveSubsystem swerveSubsystem;

//   // ========== LIMIT SWITCH ==========
//   private final DigitalInput limitSwitch;
//   private final int LIMIT_SWITCH_DIO_PORT = 0; // Change this to your DIO port
//   private boolean lastLimitSwitchState = false;
  
//   // The angle that the limit switch represents (in degrees)
//   private static final double LIMIT_SWITCH_ANGLE = 180.0;

  
//   // ========== MOTOR CONSTANTS ==========
//   private final DCMotor dcMotor = DCMotor.getNeo550(1);
//   private final int canID = 12; // Change to your CAN ID
//   private final double gearRatio = 120.0; // Change to your gear ratio
  
//   // PID Constants - TUNE THESE FOR YOUR ROBOT
//   private final double kP = 3.0; // Start with a lower value - increase if response is too slow
//   private final double kI = 0.0;
//   private final double kD = 0.0;
  
//   // Motor Configuration
//   private final boolean brakeMode = true;
//   private final double statorCurrentLimit = 40; // Amps
//   private Translation2d targetPosition = new Translation2d(0, 0);


//   //stuff
//   private boolean fieldAngleLocked = false;
//   private double lockedFieldAngleDegrees = 0.0;


//   private Transform2d turretOffset = new Transform2d(
//     new Translation2d(Units.inchesToMeters(-6), Units.inchesToMeters(-9)),  // Change these values to match your robot!
//     new Rotation2d(Math.toRadians(-58.49979400634766)) //Changing rotation stuff
//   );
//   // Turret angle limits (in degrees)
//   private static final double MIN_TURRET_ANGLE = -180.0;
//   private static final double MAX_TURRET_ANGLE = 180.0;
  
//   // Current desired angle (in degrees)
//   private double desiredAngleDegrees = 0.0;

//   // Motor controller
//   private final SparkMax motor;
//   private final RelativeEncoder encoder;
//   private final SparkSim motorSim;
//   private final SparkClosedLoopController sparkPidController;

//   // Simulation
//   private final SingleJointedArmSim turretSim;

//   /**
//    * Creates a new Turret Subsystem with limit switch.
//    */
//   public TurretSubsystem(SwerveSubsystem swerveSubsystem) {
//     this.swerveSubsystem = swerveSubsystem;
//     // Initialize limit switch
//     limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO_PORT);
//     // Initialize motor controller
//     SparkMaxConfig motorConfig = new SparkMaxConfig();
//     motor = new SparkMax(canID, MotorType.kBrushless);
//     motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

//     // Configure encoder
//     encoder = motor.getEncoder();
//     encoder.setPosition(0);

//     // Set current limits
//     motorConfig.smartCurrentLimit((int) statorCurrentLimit);

//     // Configure PID
//     sparkPidController = motor.getClosedLoopController();
//     motorConfig.closedLoop
//       .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//       .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

//     motorConfig.inverted(true);

//     // Configure Encoder Gear Ratio - CRITICAL FOR POSITION CONTROL
//     motorConfig.encoder
//       .positionConversionFactor((1.0 / gearRatio))  // Output shaft rotations per motor rotation
//       .velocityConversionFactor((1.0 / gearRatio) / 60.0); // Convert RPM to RPS
    

//     // Save configuration
//     motor.configure(
//       motorConfig,
//       ResetMode.kResetSafeParameters,
//       PersistMode.kPersistParameters
//     );
//     motorSim = new SparkSim(motor, dcMotor);

//     // Initialize simulation
//     turretSim = new SingleJointedArmSim(
//       dcMotor,
//       gearRatio,
//       0.01, // Moment of inertia
//       0.1, // Arm length (m)
//       Units.degreesToRadians(MIN_TURRET_ANGLE),
//       Units.degreesToRadians(MAX_TURRET_ANGLE),
//       false, // No gravity for horizontal turret
//       Units.degreesToRadians(0)
//     );
    
//     System.out.println("Turret initialized with CAN ID: " + canID + ", Gear Ratio: " + gearRatio);
//   }
//  // ========== FIELD-RELATIVE AIMING METHODS ==========
  
//   /**
//    * Set the target position on the field that the turret should aim at
//    * @param targetPosition Field-relative position in meters (x, y)
//    */
//   public void setTargetPosition(Translation2d targetPosition) {
//     this.targetPosition = targetPosition;
//   }
  
//   /**
//    * Set the target pose on the field that the turret should aim at
//    * This extracts the translation (x, y) from the pose
//    * @param targetPose Field-relative pose of the target
//    */
//   public void setTargetPose(Pose2d targetPose) {
//     this.targetPosition = targetPose.getTranslation();
//   }
  
//   /**
//    * Set the target position on the field using x and y coordinates
//    * @param x X coordinate in meters
//    * @param y Y coordinate in meters
//    */
//   public void setTargetPosition(double x, double y) {
//     this.targetPosition = new Translation2d(x, y);
//   }
  
//   /**
//    * Calculate the required turret angle to aim at the target position
//    * This accounts for:
//    * 1. Robot position on field
//    * 2. Robot rotation
//    * 3. Turret offset from robot center (using Transform2d)
//    * 4. Turret angle limits (-180 to 180 degrees)
//    * @return Required turret angle in degrees (robot-relative, -180 to 180)
//    */
//   public double calculateTurretAngle() {
//     if (swerveSubsystem == null) {
//       System.err.println("Warning: SwerveSubsystem not set! Cannot calculate field-relative angle.");
//       return 0.0;
//     }
    
//     // Get current robot pose from YAGSL
//     Pose2d robotPose = swerveSubsystem.getPose();
    
//     // Calculate the ACTUAL turret position on the field
//     // This transforms the turret offset by the robot's pose
//     Translation2d turretPosition = robotPose.getTranslation()
//         .plus(turretOffset.getTranslation().rotateBy(robotPose.getRotation()));
//     Rotation2d robotRotation = robotPose.getRotation();
    
//     // Calculate vector from TURRET position to target
//     Translation2d turretToTarget = targetPosition.minus(turretPosition);
    
//     // Calculate field-relative angle to target (from turret's perspective)
//     double fieldRelativeAngle = Math.atan2(turretToTarget.getY(), turretToTarget.getX());
//     fieldRelativeAngle = Math.toDegrees(fieldRelativeAngle);
    
//     // Convert to robot-relative angle
//     // Subtract robot's rotation to get the angle relative to robot's front
// double robotRelativeAngle = fieldRelativeAngle - robotRotation.getDegrees() 
//     - turretOffset.getRotation().getDegrees();    
//     // Normalize angle to -180 to 180 range
//     robotRelativeAngle = normalizeAngle(robotRelativeAngle);
    
//     // Clamp to turret limits
//     robotRelativeAngle = MathUtil.clamp(robotRelativeAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    
//     return robotRelativeAngle;
//   }
  
//     public void aimAtTarget() {
//     double targetAngle = calculateTurretAngle();
//     desiredAngleDegrees = targetAngle;
//     setAngle(targetAngle);
//   }

  
//   /**
//    * Get distance to target (from turret position, not robot center)
//    * @return Distance to target in meters
//    */
//   public double getDistanceToTarget() {
//     if (swerveSubsystem == null) {
//       return 0.0;
//     }
//     Pose2d robotPose = swerveSubsystem.getPose();
//     // Calculate actual turret position using the offset
//     Pose2d turretPose = robotPose.plus(turretOffset);
//     Translation2d turretPosition = turretPose.getTranslation();
//     return turretPosition.getDistance(targetPosition);
//   }
  
//   /**
//    * Set the turret offset from robot center
//    * @param offset Transform2d representing turret position relative to robot center
//    */
//   public void setTurretOffset(Transform2d offset) {
//     this.turretOffset = offset;
//   }
  
//   /**
//    * Set the turret offset using x and y coordinates
//    * @param x X offset in meters (positive = forward)
//    * @param y Y offset in meters (positive = left)
//    */
//   public void setTurretOffset(double x, double y) {
//     this.turretOffset = new Transform2d(new Translation2d(x, y), new Rotation2d());
//   }
  
  
//   /**
//    * Get the current turret offset
//    * @return Transform2d representing turret offset from robot center
//    */
//   public Transform2d getTurretOffset() {
//     return turretOffset;
//   }
  
//   /**
//    * Get the actual turret position on the field
//    * @return Pose2d of the turret on the field
//    */
//   public Pose2d getTurretPose() {
//     if (swerveSubsystem == null) {
//       return new Pose2d();
//     }
//     return swerveSubsystem.getPose().plus(turretOffset);
//   }


//   /**
//    * Update telemetry and handle limit switch.
//    */
//   @Override
//   public void periodic() {
//     // Check limit switch
//     boolean limitSwitchPressed = limitSwitch.get(); // Inverted because limit switches are normally open
    
//     // Detect rising edge (limit switch just pressed)
//     if (limitSwitchPressed && !lastLimitSwitchState) {
//       //resetToLimitSwitch();
//       System.out.println("Limit switch hit! Resetting turret to " + LIMIT_SWITCH_ANGLE + " degrees");
//     }
    
//     lastLimitSwitchState = limitSwitchPressed;
    
//     // Get normalized angle for telemetry
//     double normalizedAngle = normalizeAngle(Units.radiansToDegrees(getPositionRadians()));
    
//     // Telemetry


//     SmartDashboard.putBoolean("Turret/FieldAngleLocked", fieldAngleLocked);
//     SmartDashboard.putNumber("Turret/LockedFieldAngle", lockedFieldAngleDegrees);
//     SmartDashboard.putNumber("Turret/Current Angle", normalizedAngle);
//     SmartDashboard.putNumber("Turret/Current Angle (Raw)", Units.radiansToDegrees(getPositionRadians()));
//     SmartDashboard.putNumber("Turret/Current Position (Rotations)", getPosition());
//     SmartDashboard.putNumber("Turret/Desired Angle", desiredAngleDegrees);
//     SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
//     SmartDashboard.putNumber("Turret/Current", getCurrent());
//     SmartDashboard.putNumber("Turret/Voltage", getVoltage());
//     SmartDashboard.putNumber("Turret/Applied Output", motor.getAppliedOutput());
//     SmartDashboard.putBoolean("Turret/Limit Switch", limitSwitchPressed);
//   }

//   /**
//    * Update simulation.
//    */
//   @Override
//   public void simulationPeriodic() {
//     // Set input voltage from motor controller to simulation
//     turretSim.setInput(getVoltage());

//     // Update simulation by 20ms
//     turretSim.update(0.020);
//     RoboRioSim.setVInVoltage(
//       BatterySim.calculateDefaultBatteryLoadedVoltage(
//         turretSim.getCurrentDrawAmps()
//       )
//     );

//     double motorPosition = Radians.of(turretSim.getAngleRads() * gearRatio).in(Rotations);
//     double motorVelocity = RadiansPerSecond.of(
//       turretSim.getVelocityRadPerSec() * gearRatio
//     ).in(RotationsPerSecond);
//     motorSim.iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
//   }

//   // ========== LIMIT SWITCH METHODS ==========
  
//   /**
//    * Reset the encoder position when the limit switch is hit.
//    */
//   private void resetToLimitSwitch() {
//     double limitPositionRadians = Units.degreesToRadians(LIMIT_SWITCH_ANGLE);
//     double limitPositionRotations = limitPositionRadians / (2.0 * Math.PI);
//     encoder.setPosition(limitPositionRotations);
//     System.out.println("Encoder reset to: " + limitPositionRotations + " rotations (" + LIMIT_SWITCH_ANGLE + "°)");
//   }
  
//   /**
//    * Check if the limit switch is pressed.
//    * @return True if limit switch is pressed
//    */
//   public boolean isLimitSwitchPressed() {
//     return !limitSwitch.get(); // Inverted
//   }



// //Lock in position:

// /** Lock the turret to its current field-relative angle. */
// public void lockFieldAngle() {
//     if (swerveSubsystem == null) return;
//     // Current field-relative angle = robot rotation + current turret angle
//     double robotDeg = swerveSubsystem.getPose().getRotation().getDegrees();
//     double turretDeg = normalizeAngle(getPositionDegrees());
//     lockedFieldAngleDegrees = normalizeAngle(robotDeg + turretDeg);
//     fieldAngleLocked = true;
//     System.out.println("Field angle locked at: " + lockedFieldAngleDegrees + "°");
// }

// /** Unlock field angle lock — turret returns to normal robot-relative control. */
// public void unlockFieldAngle() {
//     fieldAngleLocked = false;
//     System.out.println("Field angle lock released");
// }

// public boolean isFieldAngleLocked() { return fieldAngleLocked; }

// /**
//  * When field-angle-locked, calculates the robot-relative angle needed
//  * to hold the locked field angle as the robot rotates.
//  */
// public double calculateLockedAngle() {
//     double robotDeg = swerveSubsystem.getPose().getRotation().getDegrees();
//     double robotRelative = normalizeAngle(lockedFieldAngleDegrees - robotDeg);
//     return MathUtil.clamp(robotRelative, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
// }

// /**
//  * Nudges the locked field angle (called during manual override while locked).
//  * This lets you manually steer while locked — the lock angle updates in real time.
//  */
// public void nudgeLockedAngle(double deltaDegrees) {
//     lockedFieldAngleDegrees = normalizeAngle(lockedFieldAngleDegrees + deltaDegrees);
// }

//   // ========== POSITION GETTERS ==========

//   /**
//    * Get the current position in Rotations.
//    * @return Position in Rotations
//    */
//   @Logged(name = "Position/Rotations")
//   public double getPosition() {
//     return encoder.getPosition();
//   }
  
//   /**
//    * Get the current position in Radians.
//    * @return Position in Radians
//    */
//   public double getPositionRadians() {
//     return encoder.getPosition() * 2.0 * Math.PI;
//   }
  
//   /**
//    * Get the current position in Degrees.
//    * @return Position in Degrees
//    */
//   public double getPositionDegrees() {
//     return Units.radiansToDegrees(getPositionRadians());
//   }

//   /**
//    * Get the current applied voltage.
//    * @return Applied voltage
//    */
//   @Logged(name = "Voltage")
//   public double getVoltage() {
//     return motor.getAppliedOutput() * motor.getBusVoltage();
//   }

//   /**
//    * Get the current motor current.
//    * @return Motor current in amps
//    */
//   public double getCurrent() {
//     return motor.getOutputCurrent();
//   }

//   // ========== CONTROL METHODS ==========

//   /**
//    * Set turret angle.
//    * @param angleDegrees The target angle in degrees
//    */
// public void setAngle(double angleDegrees) {
//     angleDegrees = normalizeAngle(angleDegrees);
//     angleDegrees = MathUtil.clamp(angleDegrees, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);

//     desiredAngleDegrees = angleDegrees;

//     double currentDegrees = getPositionDegrees();
//     double currentNormalized = normalizeAngle(currentDegrees);
//     double delta = normalizeAngle(angleDegrees - currentNormalized);
//     double targetDegrees = currentDegrees + delta;

//     // ===== ADD THESE LINES =====
//     // Clamp the RAW target to prevent encoder from wandering past limits
//     targetDegrees = MathUtil.clamp(targetDegrees, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
//     // If encoder has already drifted outside limits, snap it back
//     if (currentDegrees > MAX_TURRET_ANGLE) targetDegrees = MAX_TURRET_ANGLE;
//     if (currentDegrees < MIN_TURRET_ANGLE) targetDegrees = MIN_TURRET_ANGLE;
//     // ===========================

//     double positionRotations = Units.degreesToRadians(targetDegrees) / (2.0 * Math.PI);
//     sparkPidController.setReference(
//         positionRotations,
//         ControlType.kPosition,
//         ClosedLoopSlot.kSlot0
//     );
// }
  
//   /**
//    * Stop the turret.
//    */
//   public void stop() {
//     motor.stopMotor();
//     System.out.println("Turret stopped");
//   }

//   /**
//    * Check if turret is at the desired angle.
//    * Uses normalized angles for comparison.
//    * @param tolerance Tolerance in degrees
//    * @return True if within tolerance
//    */
//   public boolean atSetpoint(double tolerance) {
//     double currentAngle = normalizeAngle(Units.radiansToDegrees(getPositionRadians()));
//     double normalizedDesired = normalizeAngle(desiredAngleDegrees);
//     double error = Math.abs(normalizeAngle(currentAngle - normalizedDesired));
//     return error < tolerance;
//   }
  
//   /**
//    * Check if turret is at the desired angle with default tolerance.
//    * @return True if within 5 degrees
//    */
//   public boolean atSetpoint() {
//     return atSetpoint(5.0);
//   }

//   /**
//    * Normalize an angle to the range -180 to 180 degrees.
//    * @param angle Angle in degrees
//    * @return Normalized angle in degrees
//    */
//   private double normalizeAngle(double angle) {
//     angle = angle % 360.0;
//     if (angle > 180.0) {
//       angle -= 360.0;
//     } else if (angle < -180.0) {
//       angle += 360.0;
//     }
//     return angle;
//   }

//   // ========== COMMANDS ==========

//   public Command aimAtTargetCommand(Translation2d target) {
//     return run(() -> {
//       setTargetPosition(target);
//       aimAtTarget();
//     });
//   }
  
//   /**
//    * Creates a command to continuously aim at a target pose
//    * @param targetPose Field-relative target pose
//    * @return A command that aims the turret at the target
//    */
//   public Command aimAtTargetCommand(Pose2d targetPose) {
//     return run(() -> {
//       setTargetPose(targetPose);
//       aimAtTarget();
//     });
//   }
  
//   /**
//    * Creates a command to aim at a target and wait until on target
//    * @param target Field-relative target position (x, y in meters)
//    * @param timeoutSeconds Maximum time to wait
//    * @return A command that aims and waits
//    */
//   public Command aimAndWaitCommand(Translation2d target, double timeoutSeconds) {
//     return aimAtTargetCommand(target)
//       .until(() -> atSetpoint())
//       .withTimeout(timeoutSeconds);
//   }
  
//   /**
//    * Creates a command to aim at a target pose and wait until on target
//    * @param targetPose Field-relative target pose
//    * @param timeoutSeconds Maximum time to wait
//    * @return A command that aims and waits
//    */
//   public Command aimAndWaitCommand(Pose2d targetPose, double timeoutSeconds) {
//     return aimAtTargetCommand(targetPose)
//       .until(() -> atSetpoint())
//       .withTimeout(timeoutSeconds);
//   }





//   /**
//    * Creates a command to set the turret to a specific angle.
//    * @param angleDegrees The target angle in degrees
//    * @return A command that sets the turret to the specified angle
//    */
//   public Command setAngleCommand(double angleDegrees) {
//     return runOnce(() -> {
//       System.out.println("setAngleCommand called for " + angleDegrees + "°");
//       setAngle(angleDegrees);
//     });
//   }

//   /**
//    * Creates a command to move the turret to a specific angle and wait until reached.
//    * @param angleDegrees The target angle in degrees
//    * @return A command that moves the turret to the specified angle
//    */
//   public Command moveToAngleCommand(double angleDegrees) {
//     return run(() -> setAngle(angleDegrees))
//       .until(() -> atSetpoint())
//       .finallyDo(() -> System.out.println("moveToAngleCommand finished for " + angleDegrees + "°"));
//   }

//   /**
//    * Creates a command to stop the turret.
//    * @return A command that stops the turret
//    */
//   public Command stopCommand() {
//     return runOnce(() -> stop());
//   }
  

// /**
//  * Locks turret to a field-relative angle. Manual X/B inputs nudge the 
//  * locked angle rather than the robot-relative angle, so it "sticks" to
//  * the field. On cancel, lock is released.
//  */
// public Command fieldAngleLockCommand() {
//     return runOnce(this::lockFieldAngle)
//         .andThen(run(() -> setAngle(calculateLockedAngle())))
//         .finallyDo((interrupted) -> unlockFieldAngle());
// }


// /** * Creates a command to hold the turret at its current position.
//  * @return A command that holds the turret at its current position. Useful as a default command to prevent drift.
//  */

// public Command holdPositionCommand() {
//     final double[] heldRotations = {0};
//     return runOnce(() -> {
//         // Capture the exact encoder position ONCE when command starts
//         heldRotations[0] = encoder.getPosition();
//     }).andThen(run(() -> {
//         // Hold that exact encoder position every loop, no math, no normalization
//         sparkPidController.setReference(
//             heldRotations[0],
//             ControlType.kPosition,
//             ClosedLoopSlot.kSlot0
//         );
//     }));
// }

// /** 
//  * Updated manual command — when field-locked, nudges the locked angle.
//  * When not locked, behaves exactly as before.
//  */
// public Command manualControlCommand(java.util.function.DoubleSupplier speedSupplier, double maxSpeed) {
//     return run(() -> {
//         double speed = speedSupplier.getAsDouble();
//         if (Math.abs(speed) < 0.1) return;

//         double angleIncrement = speed * maxSpeed * 0.02;

//         if (fieldAngleLocked) {
//             // Nudge the locked field angle — turret "steers" while staying field-locked
//             nudgeLockedAngle(angleIncrement);
//             setAngle(calculateLockedAngle());
//         } else {
//             double newAngle = desiredAngleDegrees + angleIncrement;
//             setAngle(newAngle);
//         }
//     });
// }


//   /**
//    * Get the turret simulation for testing.
//    * @return The turret simulation model
//    */
//   public SingleJointedArmSim getSimulation() {
//     return turretSim;
//   }
// }
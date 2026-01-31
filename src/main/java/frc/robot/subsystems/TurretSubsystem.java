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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

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
   * Creates a new Pivot Subsystem.
   */
  public TurretSubsystem() {
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
      .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

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
      Units.degreesToRadians(-90), // Min angle (rad)
      Units.degreesToRadians(90), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(0) // Starting position (rad)
    );
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {}

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use getVoltage() for other controllers
    pivotSim.setInput(getVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        pivotSim.getCurrentDrawAmps()
      )
    );
//armsim -> pivotsim ??
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
  public double getPositionRadians()
  {
    return encoder.getPosition() % 360;
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
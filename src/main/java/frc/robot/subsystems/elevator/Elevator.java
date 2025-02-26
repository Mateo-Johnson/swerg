package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  // Hardware
  private final SparkMax motor1;
  private final SparkMax motor2;
  private final AbsoluteEncoder heightEncoder;
  private final DigitalInput lowerLimit;

  // Control
  private final PIDController pidController;
  private boolean isManualControl = false;
  private double manualSpeed = 0.0;
  private boolean softLimitsEnabled = true;
  private double lastEncoderPosition = 0.0;
  private boolean isStalled = false;
  private int stallCount = 0;
  private double targetPosition = 0.0;

  // Constants
  private static final double KP = 1.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double GRAVITY_COMPENSATION = 0.06;
  private boolean gravityCompEnabled = true;
  private static final double POSITION_TOLERANCE = 0.02;

  // Setpoints in revolutions
  private static final double[] SETPOINTS = {
    0.0,
    0.25,
    0.5,
    0.75
  }; // L1, L2, L3, L4

  // Safety limits in revolutions
  private static final double MIN_HEIGHT = 0.0; // Lowest position
  private static final double MAX_HEIGHT = 1.0; // Highest position
  private static final double MANUAL_SPEED_LIMIT = 0.8;
  private static final double STALL_THRESHOLD = 0.001; // movement threshold
  private static final int STALL_SAMPLES_THRESHOLD = 50; // 50 * 20ms = 1 second

  // State tracking
  private ElevatorState currentState = ElevatorState.IDLE;

  public enum ElevatorState {
    IDLE,
    MOVING_TO_POSITION,
    MANUAL_CONTROL,
    HOMING,
    ERROR,
    STALLED
  }

  public Elevator() {
    motor1 = new SparkMax(ElevatorConstants.rightCANId, MotorType.kBrushless);
    motor2 = new SparkMax(ElevatorConstants.leftCANId, MotorType.kBrushless);
    heightEncoder = motor1.getAbsoluteEncoder();

    // Initialize the limit switch with the provided port
    this.lowerLimit = new DigitalInput(0);

    var config = new SparkMaxConfig();
    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    // Use raw encoder counts initially
    config.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0);

    motor1.configure(
      config,
      com.revrobotics.spark.SparkMax.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkMax.PersistMode.kPersistParameters
    );

    // Configure PID controller (simpler without motion profiling)
    pidController = new PIDController(KP, KI, KD);
    pidController.setTolerance(POSITION_TOLERANCE);

    setState(ElevatorState.IDLE);
  }

  @Override
  public void periodic() {
    updateTelemetry();
    checkStallCondition();
    
    if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
      setMotorOutput(0);
      return;
    }

    // Simple control logic
    if (isManualControl) {
      // In manual mode, just apply the manual speed
      setMotorOutput(manualSpeed);
    } else if (currentState == ElevatorState.MOVING_TO_POSITION) {
      // Simple PID control based on the difference between current position and target
      double currentHeight = getHeight();
      double pidOutput = pidController.calculate(currentHeight, targetPosition);
      setMotorOutput(pidOutput);
    } else if (currentState == ElevatorState.HOMING) {
      if (lowerLimit.get()) {
        // If lower limit is triggered during homing, stop and reset
        completeHoming();
      } else {
        // Continue moving down slowly during homing
        setMotorOutput(-0.2);
      }
    } else {
      // In IDLE state, stop motors
      setMotorOutput(0);
    }

    lastEncoderPosition = getHeight();
  }

  /**
   * Enables or disables manual control mode for the elevator.
   *
   * @param enabled Whether manual control should be enabled.
   */
  public void setManualControl(boolean enabled) {
    isManualControl = enabled;
    if (enabled) {
      manualSpeed = 0.0;
      setState(ElevatorState.MANUAL_CONTROL);
    } else {
      setState(ElevatorState.IDLE);
    }
  }

  /**
   * Sets the speed for manual control of the elevator.
   *
   * @param speed The speed of the elevator, constrained by the manual speed limit.
   */
  public void setManualSpeed(double speed) {
    if (!isManualControl) return;
    
    // Limit the speed to safe values
    speed = Math.max(-MANUAL_SPEED_LIMIT, Math.min(MANUAL_SPEED_LIMIT, speed));
    
    // Apply soft limits if enabled
    if (softLimitsEnabled) {
      if ((getHeight() >= MAX_HEIGHT && speed > 0) ||
          (getHeight() <= MIN_HEIGHT && speed < 0) ||
          (lowerLimit.get() && speed < 0)) {
        speed = 0.0;
      }
    }
    
    manualSpeed = speed;
  }

  /**
   * Moves the elevator to a specified setpoint from the predefined setpoints array.
   *
   * @param setpointIndex The index of the setpoint to move to.
   */
  public void moveToSetpoint(int setpointIndex) {
    if (setpointIndex < 0 || setpointIndex >= SETPOINTS.length) {
      throw new IllegalArgumentException("Invalid setpoint index");
    }
    
    isManualControl = false;
    targetPosition = SETPOINTS[setpointIndex];
    setState(ElevatorState.MOVING_TO_POSITION);
  }
  
  /**
   * Sets a custom position target for the elevator.
   * 
   * @param position The target position in revolutions.
   */
  public void setTargetPosition(double position) {
    isManualControl = false;
    targetPosition = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, position));
    setState(ElevatorState.MOVING_TO_POSITION);
  }

  /**
   * Checks if the elevator has reached the target position.
   *
   * @return True if the elevator has reached the target position, false otherwise.
   */
  public boolean atSetpoint() {
    return Math.abs(getHeight() - targetPosition) < POSITION_TOLERANCE;
  }

  /**
   * Starts the homing process, moving the elevator down until it reaches the lower limit switch.
   */
  public void startHoming() {
    isManualControl = false;
    setState(ElevatorState.HOMING);
  }

  /**
   * Completes the homing process by resetting the encoder position to 0 when the lower limit is triggered.
   */
  public void completeHoming() {
    motor1.getEncoder().setPosition(0);
    setMotorOutput(0);
    setState(ElevatorState.IDLE);
  }

  /**
   * Emergency stops the elevator immediately and disables all motors
   */
  public void emergencyStop() {
    // Immediately stop all motion
    setMotorOutput(0);
    motor1.disable();
    motor2.disable();

    // Disable manual control
    isManualControl = false;

    // Set to error state
    setState(ElevatorState.ERROR);
  }

  /**
   * Enables or disables the soft limits, which prevent the elevator from moving beyond its safety limits.
   *
   * @param enabled Whether to enable soft limits.
   */
  public void enableSoftLimits(boolean enabled) {
    softLimitsEnabled = enabled;
  }

  /**
   * Checks if the elevator is stalled based on encoder position and motor output.
   */
  private void checkStallCondition() {
    double currentPosition = getHeight();
    boolean isMoving = Math.abs(motor1.get()) > 0.1;
    
    if (isMoving && Math.abs(currentPosition - lastEncoderPosition) < STALL_THRESHOLD) {
      stallCount++;
      if (stallCount > STALL_SAMPLES_THRESHOLD) {
        isStalled = true;
        setState(ElevatorState.STALLED);
      }
    } else {
      stallCount = 0;
      isStalled = false;
    }
  }

  /**
   * Sets the current state of the elevator subsystem.
   *
   * @param newState The new state to set.
   */
  public void setState(ElevatorState newState) {
    currentState = newState;
  }

  /**
   * Updates the telemetry displayed on the SmartDashboard.
   */
  private void updateTelemetry() {
    SmartDashboard.putString("Elevator/State", currentState.toString());
    SmartDashboard.putNumber("Elevator/Height", getHeight());
    SmartDashboard.putNumber("Elevator/Target", targetPosition);
    SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
    SmartDashboard.putBoolean("Elevator/LowerLimit", isAtLowerLimit());
    SmartDashboard.putBoolean("Elevator/Stalled", isStalled);
    SmartDashboard.putNumber("Elevator/Output", motor1.get());
    SmartDashboard.putBoolean("Elevator/GravityCompEnabled", gravityCompEnabled);
  }

  /**
   * Retrieves the current height of the elevator based on the encoder position.
   *
   * @return The current height in revolutions.
   */
  public double getHeight() {
    return heightEncoder.getPosition() / 8192; // COUNTS PER REVOLUTION
  }

  /**
   * Sets the output for the elevator motors.
   *
   * @param output The desired output speed for the motors.
   */
  private void setMotorOutput(double output) {
    if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
      output = 0;
    }

    // Apply soft limits
    if (softLimitsEnabled) {
      if (getHeight() >= MAX_HEIGHT && output > 0) output = 0;
      if (getHeight() <= MIN_HEIGHT && output < 0) output = 0;
      if (lowerLimit.get() && output < 0) output = 0;
    }

    // Add gravity compensation when enabled
    if (gravityCompEnabled && output != 0) {
      output += GRAVITY_COMPENSATION;
    }

    // Invert the output for motor direction
    output = -output;

    // Clamp final output to valid range
    output = Math.max(-1.0, Math.min(1.0, output));

    motor1.set(output);
    motor2.set(output);
  }

  /**
   * Enables or disables gravity compensation
   * @param enabled Whether gravity compensation should be enabled
   */
  public void enableGravityCompensation(boolean enabled) {
    gravityCompEnabled = enabled;
  }

  /**
   * Checks if the elevator is at the lower limit position.
   *
   * @return True if the elevator is at the lower limit, false otherwise.
   */
  public boolean isAtLowerLimit() {
    return lowerLimit.get();
  }

  /**
   * Gets the current target position.
   *
   * @return The target position in revolutions.
   */
  public double getTargetPosition() {
    return targetPosition;
  }

  /**
   * Retrieves the current state of the elevator subsystem.
   *
   * @return The current state.
   */
  public ElevatorState getCurrentState() {
    return currentState;
  }

  /**
   * Checks if the elevator is stalled.
   *
   * @return True if the elevator is stalled, false otherwise.
   */
  public boolean isStalled() {
    return isStalled;
  }

  /**
   * Clears any error or stalled condition, returning the elevator to the idle state.
   */
  public void clearError() {
    if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
      setState(ElevatorState.IDLE);
      isStalled = false;
      stallCount = 0;
    }
  }

  /**
   * Configures the PID controller for the elevator with custom PID values.
   *
   * @param p The proportional constant.
   * @param i The integral constant.
   * @param d The derivative constant.
   */
  public void configurePID(double p, double i, double d) {
    pidController.setPID(p, i, d);
  }
}
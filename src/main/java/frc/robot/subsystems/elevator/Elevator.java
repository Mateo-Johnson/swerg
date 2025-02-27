package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  // Hardware
  private final SparkMax motor1;
  private final SparkMax motor2;
  private final RelativeEncoder encoder1;
  private final RelativeEncoder encoder2;
  private final DigitalInput lowerLimit;

  // Control
  private final PIDController pidController;
  private boolean isManualControl = false;
  private double manualSpeed = 0.0;
  private double lastEncoderPosition = 0.0;
  private boolean isStalled = false;
  private int stallCount = 0;
  private double targetPosition = 0.0;
  private boolean wasLimitPressed = false; // For edge detection

  // Constants
  private static final double KP = 1.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double GRAVITY_COMPENSATION = 0.05;
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
  
  // Encoder discrepancy detection
  private static final double ENCODER_DISCREPANCY_THRESHOLD = 0.05; // Revolutions
  private boolean encoderDiscrepancyDetected = false;

  // State tracking
  private ElevatorState currentState = ElevatorState.IDLE;

  public enum ElevatorState {
    IDLE,
    MOVING_TO_POSITION,
    MANUAL_CONTROL,
    HOMING,
    ERROR,
    STALLED,
    ENCODER_ERROR
  }

  public Elevator() {
    motor1 = new SparkMax(ElevatorConstants.rightCANId, MotorType.kBrushless);
    motor2 = new SparkMax(ElevatorConstants.leftCANId, MotorType.kBrushless);
    
    // Get encoders from both motors
    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();

    // Initialize the limit switch with the provided port
    this.lowerLimit = new DigitalInput(0);

    // Configure motor1
    var config1 = new SparkMaxConfig();
    config1
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    // Set conversion factors for the encoders
    config1.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0);

    motor1.configure(
      config1,
      com.revrobotics.spark.SparkMax.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkMax.PersistMode.kPersistParameters
    );
    
    // Configure motor2 (independently, not following)
    var config2 = new SparkMaxConfig();
    config2
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    config2.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0);

    motor2.configure(
      config2,
      com.revrobotics.spark.SparkMax.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkMax.PersistMode.kPersistParameters
    );

    // Configure PID controller
    pidController = new PIDController(KP, KI, KD);
    pidController.setTolerance(POSITION_TOLERANCE);

    // Enable gravity compensation by default
    gravityCompEnabled = true;

    setState(ElevatorState.IDLE);
    
    // Initialize wasLimitPressed with current limit state
    wasLimitPressed = isAtLowerLimit();
  }

  @Override
  public void periodic() {
    // Check if limit switch was just pressed (rising edge detection)
    boolean isLimitPressed = isAtLowerLimit();
    
    // If limit switch is pressed (and wasn't before), zero the encoders
    if (isLimitPressed && !wasLimitPressed) {
      zeroEncoders();
      SmartDashboard.putBoolean("Elevator/AutoZeroed", true);
    } else {
      SmartDashboard.putBoolean("Elevator/AutoZeroed", false);
    }
    
    // Update for next cycle
    wasLimitPressed = isLimitPressed;
    
    updateTelemetry();
    checkStallCondition();
    checkEncoderDiscrepancy();
    
    if (currentState == ElevatorState.ERROR || 
        currentState == ElevatorState.STALLED || 
        currentState == ElevatorState.ENCODER_ERROR) {
      setMotorOutput(0);
      return;
    }

    // Control logic
    if (isManualControl) {
      // In manual mode, just apply the manual speed
      setMotorOutput(manualSpeed);
    } else if (currentState == ElevatorState.MOVING_TO_POSITION) {
      // PID control based on the average position
      double currentHeight = getHeight();
      double pidOutput = pidController.calculate(currentHeight, targetPosition);
      setMotorOutput(pidOutput);
    } else if (currentState == ElevatorState.HOMING) {
      if (isLimitPressed) {
        // If lower limit is triggered during homing, stop and complete homing
        completeHoming();
      } else {
        // Continue moving down slowly during homing
        setMotorOutput(-0.2);
      }
    } else {
      // In IDLE state, apply gravity compensation to hold position
      setMotorOutput(0);
    }

    lastEncoderPosition = getHeight();
  }

  /**
   * Zeros both encoders - useful when the elevator reaches a known position
   */
  public void zeroEncoders() {
    encoder1.setPosition(0);
    encoder2.setPosition(0);
    
    // Log the zeroing event
    SmartDashboard.putBoolean("Elevator/EncodersZeroed", true);
    
    // If we're currently targeting a position below where we are now,
    // update the target to be 0 since we're at the bottom
    if (targetPosition < 0) {
      targetPosition = 0;
    }
  }

  /**
   * Checks if there's a significant discrepancy between the two encoders.
   */
  private void checkEncoderDiscrepancy() {
    double encoder1Position = encoder1.getPosition();
    double encoder2Position = encoder2.getPosition();
    
    // Check if the difference between encoders exceeds the threshold
    if (Math.abs(encoder1Position - encoder2Position) > ENCODER_DISCREPANCY_THRESHOLD) {
      encoderDiscrepancyDetected = true;
      SmartDashboard.putBoolean("Elevator/EncoderDiscrepancy", true);
    } else {
      encoderDiscrepancyDetected = false;
      SmartDashboard.putBoolean("Elevator/EncoderDiscrepancy", false);
    }
    
    // Log individual encoder positions
    SmartDashboard.putNumber("Elevator/Encoder1Position", encoder1Position);
    SmartDashboard.putNumber("Elevator/Encoder2Position", encoder2Position);
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
    
    if (isAtLowerLimit() && speed < 0) {
        speed = 0.0;
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
   * Completes the homing process by setting the elevator to idle after zeroing.
   */
  public void completeHoming() {
    // No need to zero encoders here as it's automatically done in periodic when limit switch is hit
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
    
    // Add encoder discrepancy information
    SmartDashboard.putBoolean("Elevator/EncoderDiscrepancyDetected", encoderDiscrepancyDetected);
    SmartDashboard.putNumber("Elevator/EncoderDiscrepancy", 
                            Math.abs(encoder1.getPosition() - encoder2.getPosition()));
  }

  /**
   * Retrieves the current height of the elevator based on the average of both encoder positions.
   *
   * @return The current height in revolutions.
   */
  public double getHeight() {
    return (encoder1.getPosition() + encoder2.getPosition()) / 2.0;
  }

  /**
   * Gets the position from encoder 1.
   * 
   * @return The position from encoder 1 in revolutions.
   */
  public double getEncoder1Position() {
    return encoder1.getPosition();
  }
  
  /**
   * Gets the position from encoder 2.
   * 
   * @return The position from encoder 2 in revolutions.
   */
  public double getEncoder2Position() {
    return encoder2.getPosition();
  }

  /**
   * Sets the output for the elevator motors.
   *
   * @param output The desired output speed for the motors.
   */
  private void setMotorOutput(double output) {
    if (currentState == ElevatorState.ERROR || 
        currentState == ElevatorState.STALLED || 
        currentState == ElevatorState.ENCODER_ERROR) {
      output = 0;
    }

    if (isAtLowerLimit() && output < 0) {
      output = 0;
    }

    if (gravityCompEnabled) {
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
   * Checks if an encoder discrepancy has been detected.
   *
   * @return True if a significant discrepancy between encoders exists, false otherwise.
   */
  public boolean hasEncoderDiscrepancy() {
    return encoderDiscrepancyDetected;
  }

  /**
   * Clears any error or stalled condition, returning the elevator to the idle state.
   */
  public void clearError() {
    if (currentState == ElevatorState.ERROR || 
        currentState == ElevatorState.STALLED || 
        currentState == ElevatorState.ENCODER_ERROR) {
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
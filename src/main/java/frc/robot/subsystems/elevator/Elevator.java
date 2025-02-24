package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final ProfiledPIDController pidController;
  private boolean isManualControl = false;
  private double manualSpeed = 0.0;
  private boolean softLimitsEnabled = true;
  private double lastEncoderPosition = 0.0;
  private boolean isStalled = false;
  private int stallCount = 0;
  private static final double HOLD_P = 0.5;
  private double holdPosition = 0.0;

  // Constants
  private static final double KP = 1.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double MAX_VELOCITY = 2.0; // revolutions per second
  private static final double MAX_ACCELERATION = 1.5; // revolutions per second squared
  private static final double GRAVITY_COMPENSATION = 0.1; // Initial estimate, tune this value
  private boolean gravityCompEnabled = true;

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
  private static final double STALL_THRESHOLD = 0.001; // meters
  private static final int STALL_SAMPLES_THRESHOLD = 50; // 50 * 20ms = 1 second
  @SuppressWarnings("unused")
  private static final double CURRENT_LIMIT = 40.0; // amps

  // State tracking
  private ElevatorState currentState = ElevatorState.IDLE;
  @SuppressWarnings("unused")
  private double lastStateChangeTime = 0.0;

  public enum ElevatorState {
    IDLE,
    MOVING_TO_POSITION,
    MANUAL_CONTROL,
    HOMING,
    ERROR,
    STALLED,
    HOLDING
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

    // Configure PID with motion profiling
    TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    pidController = new ProfiledPIDController(KP, KI, KD, constraints);
    pidController.setTolerance(0.02); // Tolerance in revolutions

    setState(ElevatorState.IDLE);
  }

  /**
   * Periodically called to update the elevator subsystem.
   * It manages the elevator state, checks for stalling, and applies control based on the current state.
   */
  @Override
  public void periodic() {
    // UPDATE SMARTDASHBOARD CALLS
    updateTelemetry();
    checkStallCondition();
    if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
      setMotorOutput(0);
      return;
    }

    if (!isManualControl) {
      double currentHeight = getHeight();
      double output;

      if (currentState == ElevatorState.HOLDING) {
        output = (holdPosition - currentHeight) * HOLD_P;
      } else {
        output = pidController.calculate(currentHeight);
      }

      setMotorOutput(output);
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
   * Holds the elevator at its current position.
   */
  public void hold() {
    isManualControl = false;
    holdPosition = getHeight();
    pidController.setGoal(holdPosition);
    setState(ElevatorState.HOLDING);
  }

  /**
   * Sets the speed for manual control of the elevator.
   *
   * @param speed The speed of the elevator, constrained by the manual speed limit.
   */
  public void setManualSpeed(double speed) {
    if (!isManualControl) return;
    speed = Math.max(-MANUAL_SPEED_LIMIT, Math.min(MANUAL_SPEED_LIMIT, speed));
    // if (softLimitsEnabled) {
    //     if ((getHeight() >= MAX_HEIGHT && speed > 0) ||
    //         (getHeight() <= MIN_HEIGHT && speed < 0) ||
    //         (!lowerLimit.get() && speed < 0)) {
    //         speed = 0.0;
    //     }
    // }
    manualSpeed = speed;
    setMotorOutputUnchecked(manualSpeed);
  }

  /**
   * Sets the output for the elevator motors without any safety checks or limits.
   * WARNING: This function bypasses all safety features and should be used with extreme caution.
   *
   * @param output The desired output speed for the motors.
   */
  private void setMotorOutputUnchecked(double output) {
    // Check lower limit switch - prevent downward motion if triggered
    // if (lowerLimit.get() && output < 0) {
    //     output = 0;
    // }
    // Invert the output here - this is where we handle the direction
    output = -output;
    motor1.set(output);
    motor2.set(output);
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
    pidController.setGoal(SETPOINTS[setpointIndex]);
    setState(ElevatorState.MOVING_TO_POSITION);
  }

  /**
   * Checks if the elevator has reached the setpoint.
   *
   * @return True if the elevator has reached the target setpoint, false otherwise.
   */
  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  /**
   * Starts the homing process, moving the elevator down until it reaches the lower limit switch.
   */
  public void startHoming() {
    isManualControl = false;
    setState(ElevatorState.HOMING);
    setMotorOutput(-0.2); // Moving down at 20% speed until limit switch
  }

  /**
   * Completes the homing process by resetting the encoder position to 0 when the lower limit is triggered.
   */
  public void completeHoming() {
    if (currentState == ElevatorState.HOMING && lowerLimit.get()) {
      motor1.getEncoder().setPosition(0);
      setState(ElevatorState.IDLE);
    }
  }

  /**
   * Emergency stops the elevator immediately and disables all motors
   */
  public void emergencyStop() {
    // Immediately stop all motion
    setMotorOutputUnchecked(0);
    motor1.disable();
    motor2.disable();

    // Disable manual control and PID
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
    if (currentState != newState) {
      currentState = newState;
      lastStateChangeTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
  }

  /**
   * Updates the telemetry displayed on the SmartDashboard.
   * Displays the elevator's current state, height, target, and other relevant information.
   */
  private void updateTelemetry() {
    SmartDashboard.putString("Elevator/State", currentState.toString());
    SmartDashboard.putNumber("Elevator/Height", getHeight());
    SmartDashboard.putNumber("Elevator/Target", getCurrentSetpoint());
    SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
    SmartDashboard.putBoolean("Elevator/LowerLimit", isAtLowerLimit());
    SmartDashboard.putBoolean("Elevator/Stalled", isStalled);
    SmartDashboard.putNumber("Elevator/Output", motor1.get());
    SmartDashboard.putBoolean("Elevator/GravityCompEnabled", gravityCompEnabled);
    SmartDashboard.putNumber("Elevator/GravityCompValue", GRAVITY_COMPENSATION);
  }

  /**
   * Retrieves the current height of the elevator based on the encoder position.
   *
   * @return The current height in meters.
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

    // Invert the output here - this is where we handle the direction
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
   * Retrieves the current setpoint that the elevator is moving towards.
   *
   * @return The current setpoint in meters.
   */
  public double getCurrentSetpoint() {
    return pidController.getGoal().position;
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
    var config = new SparkMaxConfig();
    config.closedLoop
      .pid(p, i, d);
    motor1.configure(
      config,
      com.revrobotics.spark.SparkMax.ResetMode.kNoResetSafeParameters,
      com.revrobotics.spark.SparkMax.PersistMode.kNoPersistParameters
    );
  }

  /**
   * Configures the motion profile constraints for the elevator's motion (max velocity and acceleration).
   *
   * @param maxVelocity The maximum velocity in meters per second.
   * @param maxAcceleration The maximum acceleration in meters per second squared.
   */
  public void configureMotionConstraints(double maxVelocity, double maxAcceleration) {
    pidController.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }

  /**
   * Configures the encoder conversion factor for the elevator to translate encoder ticks to meters.
   *
   * @param metersPerRotation The number of meters per encoder rotation.
   */
  public void configureEncoderConversion(double metersPerRotation) {
    var config = new SparkMaxConfig();
    config.encoder
      .positionConversionFactor(metersPerRotation)
      .velocityConversionFactor(metersPerRotation);
    motor1.configure(
      config,
      com.revrobotics.spark.SparkMax.ResetMode.kNoResetSafeParameters,
      com.revrobotics.spark.SparkMax.PersistMode.kNoPersistParameters
    );
  }
}
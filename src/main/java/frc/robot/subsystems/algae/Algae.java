package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

// Full range of motion is from 0 (top) to ~-3.38 (bottom)
public class Algae extends SubsystemBase {
    // States enum for the flipper arm
    public enum STATES {
        MOVING_TO_SETPOINT,
        AT_SETPOINT,
        IDLE
    }

    // Constants
    private static final int MOTOR_ID = 43;
    private static final double POSITION_TOLERANCE = 0.1; // Tolerance for considering "at setpoint"
    private static final double VELOCITY_TOLERANCE = 0.1; // Velocity tolerance for at setpoint
    private static final double CURRENT_THRESHOLD = 25.0; // Amps - threshold for detecting ball
    private static final double SECURE_GRIP_MULTIPLIER = 1.5; // Multiplier for securing grip
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;

    // Hardware components
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    // WPILib PID Controller
    private final PIDController pidController;

    // State variables
    private STATES currentState = STATES.IDLE;
    private double targetPosition = 0.0;
    private double currentPosition = 0.0;
    private double startPosition = 0.0;
    private double startTime = 0.0;
    private double moveEndTime = 0.0;
    private boolean isMoving = false;
    private boolean hasAlgae = false;
    private double motorOutput = 0.0;

    // Constructor
    public Algae() {
        // Initialize motor
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        
        // Get encoder
        encoder = motor.getEncoder();
        
        pidController = new PIDController(0.2, 0, 0);
        pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        
        // Reset encoder
        resetEncoder();
    }

    @Override
    public void periodic() {
        // Update current position
        currentPosition = encoder.getPosition();
        SmartDashboard.putNumber("ALGAE/POSITION", currentPosition);
        
        // Check for ball presence based on current draw
        checkForAlgae();
        
        // State machine logic
        switch (currentState) {
            case MOVING_TO_SETPOINT:
                handleMovingToSetpoint();
                break;
            case AT_SETPOINT:
                handleAtSetpoint();
                break;
            case IDLE:
                // Do nothing in idle state
                motor.set(0);
                break;
        }
    }

    public void stop() {
        motor.set(0);
    }

    /**
     * Handle the MOVING_TO_SETPOINT state logic
     */
    private void handleMovingToSetpoint() {
        double calculatedPosition;
        
        if (isMoving) {
            // If we're performing a timed move
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime >= moveEndTime) {
                // Timed move is complete
                isMoving = false;
                calculatedPosition = targetPosition;
            } else {
                // Calculate intermediate position based on time
                double timeProgress = (currentTime - startTime) / (moveEndTime - startTime);
                calculatedPosition = startPosition + (targetPosition - startPosition) * timeProgress;
            }
        } else {
            calculatedPosition = targetPosition;
        }
        
        // Calculate motor output using WPILib PID
        motorOutput = pidController.calculate(currentPosition, calculatedPosition);
        
        // Clamp output within bounds
        motorOutput = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, motorOutput));
        
        // Set motor output
        motor.set(motorOutput);
        
        // Check if we've reached the target
        if (pidController.atSetpoint()) {
            currentState = STATES.AT_SETPOINT;
        }
    }

    /**
     * Handle the AT_SETPOINT state logic
     */
    private void handleAtSetpoint() {
        // Calculate motor output to maintain position
        motorOutput = pidController.calculate(currentPosition, targetPosition);
        
        // Clamp output within bounds
        motorOutput = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, motorOutput));
        
        // Set motor output
        motor.set(motorOutput);
        
        // Check if we've drifted from the target
        if (!pidController.atSetpoint()) {
            currentState = STATES.MOVING_TO_SETPOINT;
        }
    }

    /**
     * Check for ball presence based on current draw
     */
    private void checkForAlgae() {
        double currentDraw = motor.getOutputCurrent();
        
        // Detect ball based on current threshold
        boolean previousHasAlgae = hasAlgae;
        hasAlgae = currentDraw > CURRENT_THRESHOLD;
        
        // If we just detected a ball, secure the grip
        if (hasAlgae && !previousHasAlgae) {
            secureGrip();
        }
    }

    /**
     * Apply additional force to secure the ball
     */
    private void secureGrip() {
        if (motorOutput > 0) {
            // Only increase grip if we're closing the gripper
            double secureOutput = Math.min(motorOutput * SECURE_GRIP_MULTIPLIER, MAX_OUTPUT);
            motor.set(secureOutput);
        }
    }

    /**
     * Set the target position and move to it
     * @param position The target position in encoder units
     */
    public void setTargetPosition(double position) {
        targetPosition = position;
        pidController.setSetpoint(position);
        currentState = STATES.MOVING_TO_SETPOINT;
        isMoving = false;
    }

    /**
     * Move to position over a specified time
     * @param position The target position in encoder units
     * @param seconds The time to take for the move in seconds
     */
    public void moveToPositionOverTime(double position, double seconds) {
        targetPosition = position;
        startPosition = currentPosition;
        startTime = Timer.getFPGATimestamp();
        moveEndTime = startTime + seconds;
        currentState = STATES.MOVING_TO_SETPOINT;
        isMoving = true;
    }

    /**
     * Set the state to IDLE, stopping motion
     */
    public void setIdle() {
        motor.set(0);
        currentState = STATES.IDLE;
        isMoving = false;
    }

    /**
     * Reset the encoder to zero
     */
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    /**
     * Get the current state of the flipper arm
     * @return The current state
     */
    public STATES getState() {
        return currentState;
    }

    /**
     * Get the current position of the flipper arm
     * @return The current position in encoder units
     */
    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get the target position of the flipper arm
     * @return The target position in encoder units
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Check if the flipper arm has a ball
     * @return True if a ball is detected, false otherwise
     */
    public boolean hasAlgae() {
        return hasAlgae;
    }

    /**
     * Get the current draw of the motor
     * @return The current draw in amps
     */
    public double getCurrentDraw() {
        return motor.getOutputCurrent();
    }

    /**
     * Update the PID constants
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    public void setPIDConstants(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }
    
    /**
     * Reset the PID controller's accumulated integral term
     */
    public void resetPID() {
        pidController.reset();
    }
}
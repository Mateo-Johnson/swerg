package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
    // Hardware
    private final SparkMax motor1;
    private final MotorController motor2;
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
    
    // Constants
    private static final double KP = 1.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double MAX_VELOCITY = 2.0; // meters per second
    private static final double MAX_ACCELERATION = 1.5; // meters per second squared
    
    // Setpoints in meters
    private static final double[] SETPOINTS = {0.0, 0.5, 1.0, 1.5}; // L1, L2, L3, L4
    
    // Safety limits
    private static final double MIN_HEIGHT = 0.0;  // Lowest position
    private static final double MAX_HEIGHT = 1.6;  // Highest position
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
        STALLED
    }

    public Elevator(
        SparkMax motor1,
        MotorController motor2,
        AbsoluteEncoder heightEncoder,
        DigitalInput lowerLimit) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.heightEncoder = heightEncoder;
        this.lowerLimit = lowerLimit;
        
        // Configure the motor controller
        var config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            
        config.encoder
            .positionConversionFactor(0.001)
            .velocityConversionFactor(0.001);
            
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(KP, KI, KD);
            
        motor1.configure(
            config,
            com.revrobotics.spark.SparkMax.ResetMode.kResetSafeParameters,
            com.revrobotics.spark.SparkMax.PersistMode.kPersistParameters
        );
        
        // Configure PID with motion profiling
        TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        pidController = new ProfiledPIDController(KP, KI, KD, constraints);
        pidController.setTolerance(0.02);
        
        // Initialize state
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
        if (!isManualControl && !lowerLimit.get()) {
            double currentHeight = getHeight();
            double output = pidController.calculate(currentHeight);
            setMotorOutput(output);
        }
        lastEncoderPosition = getHeight();
    }

    // Manual Control Methods
    public void setManualControl(boolean enabled) {
        isManualControl = enabled;
        if (enabled) {
            manualSpeed = 0.0;
            setState(ElevatorState.MANUAL_CONTROL);
        } else {
            setState(ElevatorState.IDLE);
        }
    }

    public void setManualSpeed(double speed) {
        if (!isManualControl) return;
        speed = Math.max(-MANUAL_SPEED_LIMIT, Math.min(MANUAL_SPEED_LIMIT, speed));
        if (softLimitsEnabled) {
            if ((getHeight() >= MAX_HEIGHT && speed > 0) ||
                (getHeight() <= MIN_HEIGHT && speed < 0) ||
                (!lowerLimit.get() && speed < 0)) {
                speed = 0.0;
            }
        }
        manualSpeed = speed;
        setMotorOutput(manualSpeed);
    }

    // Setpoint Control Methods
    public void moveToSetpoint(int setpointIndex) {
        if (setpointIndex < 0 || setpointIndex >= SETPOINTS.length) {
            throw new IllegalArgumentException("Invalid setpoint index");
        }
        isManualControl = false;
        pidController.setGoal(SETPOINTS[setpointIndex]);
        setState(ElevatorState.MOVING_TO_POSITION);
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    // Homing Methods
    public void startHoming() {
        isManualControl = false;
        setState(ElevatorState.HOMING);
        setMotorOutput(-0.2); // Moving down at 20% speed until limit switch
    }

    public void completeHoming() {
        if (currentState == ElevatorState.HOMING && lowerLimit.get()) {
            motor1.getEncoder().setPosition(0);
            setState(ElevatorState.IDLE);
        }
    }

    // Safety Features
    public void enableSoftLimits(boolean enabled) {
        softLimitsEnabled = enabled;
    }

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

    // State Management
    public void setState(ElevatorState newState) {
        if (currentState != newState) {
            currentState = newState;
            lastStateChangeTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
    }

    // Telemetry
    private void updateTelemetry() {
        SmartDashboard.putString("Elevator/State", currentState.toString());
        SmartDashboard.putNumber("Elevator/Height", getHeight());
        SmartDashboard.putNumber("Elevator/Target", getCurrentSetpoint());
        SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
        SmartDashboard.putBoolean("Elevator/LowerLimit", isAtLowerLimit());
        SmartDashboard.putBoolean("Elevator/Stalled", isStalled);
        SmartDashboard.putNumber("Elevator/Output", motor1.get());
    }

    // Utility Methods
    public double getHeight() {
        return heightEncoder.getPosition();
    }

    private void setMotorOutput(double output) {
        if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
            output = 0;
        }
        // Apply soft limits
        if (softLimitsEnabled) {
            if (getHeight() >= MAX_HEIGHT && output > 0) output = 0;
            if (getHeight() <= MIN_HEIGHT && output < 0) output = 0;
            if (!lowerLimit.get() && output < 0) output = 0;
        }
        // Invert the output here - this is where we handle the direction
        output = -output;
        motor1.set(output);
        motor2.set(output);
    }

    // Status Methods
    public boolean isAtLowerLimit() {
        return !lowerLimit.get();
    }

    public double getCurrentSetpoint() {
        return pidController.getGoal().position;
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public boolean isStalled() {
        return isStalled;
    }

    public void clearError() {
        if (currentState == ElevatorState.ERROR || currentState == ElevatorState.STALLED) {
            setState(ElevatorState.IDLE);
            isStalled = false;
            stallCount = 0;
        }
    }

    // Configuration Methods
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

    public void configureMotionConstraints(double maxVelocity, double maxAcceleration) {
        pidController.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

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
package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    // Motor and encoder configuration
    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder masterEncoder;
    private final RelativeEncoder followerEncoder;
    
    // Position control
    private final PIDController pid;
    private final DigitalInput limitSwitch;
    
    // Configuration constants
    private static final double GRAVITY_COMPENSATION = 0.05;
    private static final double MAX_MANUAL_SPEED = 0.7;
    private static final double MANUAL_DEADBAND = 0.1;
    private static final double POSITION_TOLERANCE = 0.05; // Tighter tolerance
    
    // Motor and sensor IDs
    private static final int MASTER_MOTOR_ID = ElevatorConstants.rightCANId;
    private static final int FOLLOWER_MOTOR_ID = ElevatorConstants.leftCANId;
    private static final int LIMIT_SWITCH_PORT = 0;
    
    // State management
    public enum ElevatorState {
        IDLE,
        MOVING_TO_POSITION,
        MANUAL_CONTROL,
        AT_SETPOINT,
        ERROR
    }

    // PID Gains - more aggressive tuning
    private static final double P_GAIN = 0.038;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 0;

    private ElevatorState currentState = ElevatorState.IDLE;
    private ElevatorState previousState = ElevatorState.IDLE;

    private static final int maxHeight = 50;
    
    // Control flags
    private boolean isManualControl = false;
    private double targetPosition = 0.0;
    private double pidOutput = 0.0;

    public Elevator() {
        // Motor initialization
        masterMotor = new SparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        
        // Encoder setup
        masterEncoder = masterMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();
        
        // Limit switch
        limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        
        // PID Controller with more comprehensive gains
        pid = new PIDController(P_GAIN, I_GAIN, D_GAIN);
        pid.setTolerance(POSITION_TOLERANCE);
        
        // Configure motors
        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.inverted(false)
                   .idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(40);
        
        masterMotor.configure(masterConfig, 
                             SparkMax.ResetMode.kResetSafeParameters, 
                             SparkMax.PersistMode.kPersistParameters);
    
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake)
                     .smartCurrentLimit(40)
                     .follow(masterMotor, true);
        
        followerMotor.configure(followerConfig, 
                              SparkMax.ResetMode.kResetSafeParameters, 
                              SparkMax.PersistMode.kPersistParameters);
    }

    // State Management Methods
    public void setState(ElevatorState newState) {
        previousState = currentState;
        currentState = newState;
    }

    public double getHeightPercentage() {
        return getPosition() / maxHeight;
    }

    // Position Control Methods
    public void setPosition(double position) {
        isManualControl = false;
        targetPosition = position;
        setState(ElevatorState.MOVING_TO_POSITION);
        pid.setSetpoint(position);
    }

    public void manualControl(double speed) {
        isManualControl = true;
        setState(ElevatorState.MANUAL_CONTROL);
        
        if (Math.abs(speed) < MANUAL_DEADBAND) {
            speed = 0.0;
        }
        
        speed = Math.max(-MAX_MANUAL_SPEED, Math.min(speed, MAX_MANUAL_SPEED));
        
        if (isAtLowerLimit() && speed < 0) {
            speed = 0;
        }
        
        double output = speed;
        if (speed >= 0) {
            output += GRAVITY_COMPENSATION;
        }
        
        masterMotor.set(output);
    }

    // Periodic Update Method
    @Override
    public void periodic() {
        resetEncodersIfAtLimit();

        // Position control logic
        if (currentState == ElevatorState.MOVING_TO_POSITION) {
            double currentPosition = getPosition();
            
            // Recalculate PID output
            pidOutput = pid.calculate(currentPosition, targetPosition);
            
            // Apply fixed gravity compensation regardless of direction
            double motorOutput = pidOutput + GRAVITY_COMPENSATION;
            
            // Clamp output
            motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
            
            // Set motor output
            masterMotor.set(motorOutput);

            // Check if at setpoint
            if (pid.atSetpoint()) {
                setState(ElevatorState.AT_SETPOINT);
                masterMotor.set(GRAVITY_COMPENSATION); // Hold position
            }

            // Logging for debugging
            updateSmartDashboardPositionData(currentPosition);
        }

        // General elevator logging
        updateSmartDashboardGeneralData();
    }

    private void updateSmartDashboardPositionData(double currentPosition) {
        SmartDashboard.putNumber("ElevatorPID/Current Position", currentPosition);
        SmartDashboard.putNumber("ElevatorPID/Target Position", targetPosition);
        SmartDashboard.putNumber("ElevatorPID/Position Error", pid.getPositionError());
        SmartDashboard.putNumber("ElevatorPID/PID Output", pidOutput);
        SmartDashboard.putNumber("ElevatorPID/Motor Output", masterMotor.getAppliedOutput());
    }

    private void updateSmartDashboardGeneralData() {
        SmartDashboard.putNumber("Elevator/Position", getPosition());
        SmartDashboard.putBoolean("Elevator/Lower Limit", isAtLowerLimit());
        SmartDashboard.putString("Elevator/State", currentState.toString());
    }

    // Utility Methods
    public void stop() {
        manualControl(0.0);
    }

    public void moveUp() { 
        manualControl(MAX_MANUAL_SPEED); 
    }

    public void moveDown() { 
        manualControl(-MAX_MANUAL_SPEED); 
    }

    public void resetEncoders() {
        masterEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    public void resetEncodersIfAtLimit() {
        if (isAtLowerLimit()) {
            resetEncoders();
        }
    }

    // Getter Methods
    public double getPosition() { 
        return masterEncoder.getPosition(); 
    }

    public double getFollowerPosition() { 
        return followerEncoder.getPosition(); 
    }

    public boolean isAtLowerLimit() { 
        return limitSwitch.get(); 
    }

    public boolean areEncodersSynchronized(double toleranceRotations) {
        return Math.abs(masterEncoder.getPosition() - followerEncoder.getPosition()) < toleranceRotations;
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public ElevatorState getPreviousState() {
        return previousState;
    }

    public boolean isManualControlMode() { 
        return isManualControl; 
    }
}
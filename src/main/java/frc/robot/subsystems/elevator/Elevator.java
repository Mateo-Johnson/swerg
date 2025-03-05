package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder masterEncoder;
    private final RelativeEncoder followerEncoder;
    private final PIDController pid;
    private final DigitalInput limitSwitch;
    
    // Constants remain unchanged
    private final double GRAVITY_COMPENSATION = 0.05;
    private final double MAX_MANUAL_SPEED = 0.7;
    private final double MANUAL_DEADBAND = 0.1;
    private final int MASTER_MOTOR_ID = ElevatorConstants.rightCANId;
    private final int FOLLOWER_MOTOR_ID = ElevatorConstants.leftCANId;
    private final int LIMIT_SWITCH_PORT = 0;
    private boolean isManualControl = false;

    public enum ElevatorState {
        IDLE,
        MOVING_TO_POSITION,
        MANUAL_CONTROL,
        ERROR,
        STALLED
    }

    private ElevatorState currentState = ElevatorState.IDLE;
    private ElevatorState previousState = ElevatorState.IDLE;

    public Elevator() {
        // Initialize motors
        masterMotor = new SparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        
        // Get encoders
        masterEncoder = masterMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();
        
        // Initialize limit switch
        limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        
        // Initialize PID
        pid = new PIDController(0.008, 0, 0); //0.0085
        
        // Configure motors using new configuration system
        configureMotors();
    }

    // Existing getter methods remain unchanged
    public double getPosition() { 
        return masterEncoder.getPosition(); 
    }

    public double getFollowerPosition() { 
        return followerEncoder.getPosition(); 
    
    }

    public boolean areEncodersSynchronized(double toleranceRotations) {
        return Math.abs(masterEncoder.getPosition() - followerEncoder.getPosition()) < toleranceRotations;
    }

    public boolean isAtLowerLimit() { 
        return limitSwitch.get(); 
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public ElevatorState getPreviousState() {
        return previousState;
    }

    protected void setState(ElevatorState newState) {
        previousState = currentState;
        currentState = newState;
    }

    private void configureMotors() {
        // Create configuration for master motor
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.inverted(false)
                   .idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(40);
        
        // Apply configuration to master motor
        masterMotor.configure(masterConfig, 
                             SparkMax.ResetMode.kResetSafeParameters, 
                             SparkMax.PersistMode.kPersistParameters);
    
        // Create configuration for follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake)
                     .smartCurrentLimit(40)
                     .follow(masterMotor, true);
        
        // Apply configuration to follower motor
        followerMotor.configure(followerConfig, 
                              SparkMax.ResetMode.kResetSafeParameters, 
                              SparkMax.PersistMode.kPersistParameters);
    
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

    public void moveUp() { manualControl(MAX_MANUAL_SPEED); }
    public void moveDown() { manualControl(-MAX_MANUAL_SPEED); }
    
    public void stop() {
        manualControl(0.0);
    }

    public void setPosition(double targetPosition) {
        isManualControl = false;
        setState(ElevatorState.MOVING_TO_POSITION);
        
        if (isAtLowerLimit() && targetPosition < 0.1) {
            masterMotor.set(0);
            return;
        }
        
        double pidOutput = pid.calculate(getPosition(), targetPosition);
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        motorOutput = motorOutput * 1.2;
        
        masterMotor.set(motorOutput);
    }

    public boolean isManualControlMode() { return isManualControl; }

    @Override
    public void periodic() {
        resetEncodersIfAtLimit();

        // SmartDashboard prints
        SmartDashboard.putNumber("Elevator/Position", getPosition());
        SmartDashboard.putBoolean("Elevator/Lower Limit", isAtLowerLimit());
        SmartDashboard.putString("Elevator/State", currentState.toString());
    }
}
package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralAlgaeConstants;

public class Algae extends SubsystemBase {
    // Subsystem states
    public enum AlgaeState {
        IDLE,
        INTAKE,
        OUTTAKE,
        STORE
    }

    // Hardware
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    
    // Current state
    private AlgaeState currentState = AlgaeState.IDLE;

    // Current limiting
    private static final int SCL = CoralAlgaeConstants.SCL; // Smart Current Limit
    private static final int FCL = CoralAlgaeConstants.FCL; // Free Current Limit
    
    // Motor speeds
    private static final double intakeSpeed = CoralAlgaeConstants.algaeIntakeSpeed;
    private static final double outtakeSpeed = CoralAlgaeConstants.algaeOuttakeSpeed;
    private static final double storeSpeed = CoralAlgaeConstants.algaeStoreSpeed; // Slower speed for storing
    
    // Game piece detection
    private boolean gamePresent = CoralAlgaeConstants.gamePresent;
    private double highCurrentStartTime = CoralAlgaeConstants.highCurrentStartTime;
    private boolean motorStartupIgnore = CoralAlgaeConstants.motorStartupIgnore;
    private double motorStartTime = CoralAlgaeConstants.motorStartTime;
    private static final double ignore_time = CoralAlgaeConstants.ignoreTime; // 500ms to ignore initial startup current spike
    private static final double debounce = CoralAlgaeConstants.debounce; // 100ms debounce for current detection
    
    public Algae() {
        // Initialize motors
        this.leftMotor = CoralAlgaeConstants.leftCoral;
        this.rightMotor = CoralAlgaeConstants.rightCoral;

        // Configure left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(SCL, FCL);
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.inverted(false);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, null);

        // Configure right motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(SCL, FCL);
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.inverted(true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, null);
    }
    
    @Override
    public void periodic() {
        // Run motors based on current state
        switch (currentState) {
            case INTAKE:
                runMotors(intakeSpeed);
                // Update game piece detection when intaking
                updateGamePieceDetection();
                break;
            case OUTTAKE:
                runMotors(outtakeSpeed);
                break;
            case STORE:
                runMotors(storeSpeed);
                break;
            case IDLE:
            default:
                stopMotors();
                break;
        }
        
        // Update dashboard with debug info
        updateDashboard();
    }
    
    /**
     * Set the subsystem to intake mode
     */
    public void intake() {
        // If transitioning from IDLE state, reset timers for current spike detection
        if (currentState == AlgaeState.IDLE) {
            motorStartTime = 0;
            motorStartupIgnore = true;
        }
        currentState = AlgaeState.INTAKE;
    }
    
    /**
     * Set the subsystem to outtake mode
     */
    public void outtake() {
        // Reset game piece detection when ejecting
        resetGamePieceDetection();
        currentState = AlgaeState.OUTTAKE;
    }
    
    /**
     * Set the subsystem to store mode (slow intake)
     */
    public void store() {
        currentState = AlgaeState.STORE;
    }
    
    /**
     * Stop all motors and set state to IDLE
     */
    public void stop() {
        currentState = AlgaeState.IDLE;
        // Do NOT reset gamePresent when stopping - only when ejecting
    }
    
    /**
     * Get the current state of the subsystem
     * @return Current AlgaeState
     */
    public AlgaeState getState() {
        return currentState;
    }
    
    /**
     * Run both motors at the specified speed
     * @param speed Speed to run motors at (-1.0 to 1.0)
     */
    private void runMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(0.25 * speed);
    }
    
    /**
     * Stop both motors
     */
    private void stopMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
    
    /**
     * Updates the game piece detection status based on motor current
     * Only sets gamePresent to true, never to false (except via resetGamePieceDetection)
     */
    private void updateGamePieceDetection() {
        // If we already have a game piece, don't change the state
        if (gamePresent) {
            return;
        }

        // We'll use the left motor for detection since it's our main reference
        double currentDraw = getLeftCurrentDraw();
        double currentTime = Timer.getFPGATimestamp();
        
        // Handle startup current spike ignore logic
        if (motorStartupIgnore && currentState != AlgaeState.IDLE) {
            // If we're ignoring startup and motors are running
            if (motorStartTime == 0) {
                // First time we've seen motors running since stop
                motorStartTime = currentTime;
            } else if (currentTime - motorStartTime > ignore_time) {
                // We've waited long enough, stop ignoring
                motorStartupIgnore = false;
            }
        }
        
        // Only detect game pieces when we're not ignoring startup spikes
        if (!motorStartupIgnore) {
            // Current-based detection with debouncing
            if (currentDraw >= CoralAlgaeConstants.currentThreshold) {
                if (highCurrentStartTime == 0) {
                    // Start timing how long we see high current
                    highCurrentStartTime = currentTime;
                } else if (currentTime - highCurrentStartTime > debounce) {
                    // We've seen high current for enough time
                    gamePresent = true;
                    
                    // Automatically transition to STORE state if we detect a game piece
                    if (currentState == AlgaeState.INTAKE) {
                        store();
                    }
                }
            } else {
                // Current is below threshold, reset timing
                highCurrentStartTime = 0;
            }
        }
    }
    
    /**
     * Reset the game piece detection state and the ignore timer.
     * This should be called when starting to eject a game piece.
     */
    public void resetGamePieceDetection() {
        gamePresent = false;
        motorStartupIgnore = true;
        motorStartTime = 0;
        highCurrentStartTime = 0;
    }
    
    /**
     * @return True if a game piece is detected in the system
     */
    public boolean hasGamePiece() {
        return gamePresent;
    }
    
    /**
     * Get the current draw from the left motor
     * @return Current in amps
     */
    public double getLeftCurrentDraw() {
        return leftMotor.getOutputCurrent();
    }
    
    /**
     * Get the current draw from the right motor
     * @return Current in amps
     */
    public double getRightCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }
    
    /**
     * Update the SmartDashboard with debug information
     */
    private void updateDashboard() {
        SmartDashboard.putString("ALGAE/State", currentState.toString());
        SmartDashboard.putNumber("ALGAE/Left Motor Current", getLeftCurrentDraw());
        SmartDashboard.putBoolean("ALGAE/Has Game Piece", hasGamePiece());
    }
}
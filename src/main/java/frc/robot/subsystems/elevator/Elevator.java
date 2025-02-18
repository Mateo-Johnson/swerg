package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

    // LOGGING STUFF
    private PrintWriter logWriter;
    private boolean isLogging = false;
    @SuppressWarnings("unused")
    private static final String USB_PATH = "/media/sda1/"; // Common USB mount point on RoboRIO
    private String loggingPath;
    private static final int MAX_LOG_SIZE_MB = 100; // Maximum log file size
    private long logStartTime;

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
    
    // Setpoints in revolutions
    private static final double[] SETPOINTS = {0.0, 0.25, 0.5, 0.75}; // L1, L2, L3, L4
    
    // Safety limits in revolutions
    private static final double MIN_HEIGHT = 0.0;  // Lowest position
    private static final double MAX_HEIGHT = 1.0;  // Highest position
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

        // LOGGING
        if (isLogging) {
            logData("");
        }
        
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
        if (!lowerLimit.get() && output < 0) {
            output = 0;
        }
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
            if (!lowerLimit.get() && output < 0) output = 0;
        }
        // Invert the output here - this is where we handle the direction
        output = -output;
        motor1.set(output);
        motor2.set(output);
    }

    /**
     * Checks if the elevator is at the lower limit position.
     *
     * @return True if the elevator is at the lower limit, false otherwise.
     */
    public boolean isAtLowerLimit() {
        return !lowerLimit.get();
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

       /**
     * Finds and returns the path to the USB drive, or falls back to RoboRIO storage
     */
    private String findLoggingPath() {
        // Check common USB mount points
        String[] usbPaths = {
            "/media/sda1/",
            "/media/sdb1/",
            "/media/usb0/",
            "/media/usb1/"
        };

        for (String path : usbPaths) {
            File usbDir = new File(path);
            if (usbDir.exists() && usbDir.canWrite()) {
                // Create a logs directory on the USB if it doesn't exist
                File logsDir = new File(path + "logs");
                if (!logsDir.exists()) {
                    logsDir.mkdirs();
                }
                System.out.println("USB drive found at: " + path);
                return logsDir.getAbsolutePath() + "/";
            }
        }

        // Fallback to RoboRIO if no USB is found
        System.out.println("No USB drive found, falling back to RoboRIO storage");
        Path rioPath = Paths.get(Filesystem.getOperatingDirectory().getPath(), "logs");
        if (!rioPath.toFile().exists()) {
            rioPath.toFile().mkdirs();
        }
        return rioPath.toString() + "/";
    }

    /**
     * Starts logging elevator data to the USB drive or RoboRIO
     */
    public void startLogging() {
        try {
            // Find appropriate logging location
            loggingPath = findLoggingPath();
            
            // Create filename with timestamp
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
            String timestamp = sdf.format(new Date());
            String filename = loggingPath + "elevator_log_" + timestamp + ".csv";
            
            // Check available space
            File logDir = new File(loggingPath);
            long usableSpace = logDir.getUsableSpace();
            if (usableSpace < MAX_LOG_SIZE_MB * 1024 * 1024) {
                System.err.println("Warning: Low storage space for logging");
                // Could implement auto-cleanup of old logs here
            }
            
            // Create CSV file with headers
            logWriter = new PrintWriter(new FileWriter(filename, true)); // true for append mode
            logWriter.println("Timestamp,State,Height,TargetHeight,Output,Current1,Current2,Voltage1,Voltage2," +
                            "Temperature1,Temperature2,LowerLimit,IsStalled,StallCount,ManualControl,SoftLimits," +
                            "Events,StorageLocation");
            
            isLogging = true;
            logStartTime = System.currentTimeMillis();
            logData("LOGGING_STARTED," + loggingPath);
            
            System.out.println("Logging started at: " + filename);
            
        } catch (IOException e) {
            System.err.println("Failed to start logging: " + e.getMessage());
            // Try fallback to RoboRIO if USB fails
            if (loggingPath.startsWith("/media/")) {
                loggingPath = Paths.get(Filesystem.getOperatingDirectory().getPath(), "logs").toString() + "/";
                startLogging(); // Recursive call with RoboRIO path
            }
        }
    }

    /**
     * Safely stops logging and closes the file
     */
    public void stopLogging() {
        if (isLogging && logWriter != null) {
            logData("LOGGING_STOPPED,");
            logWriter.flush();
            logWriter.close();
            isLogging = false;
            
            // Calculate and log session duration
            long duration = System.currentTimeMillis() - logStartTime;
            System.out.println("Logging session completed. Duration: " + (duration / 1000) + " seconds");
        }
    }

    /**
     * Logs current elevator data with storage location tracking
     */
    private void logData(String event) {
        if (!isLogging || logWriter == null) return;
        
        try {
            StringBuilder sb = new StringBuilder();
            double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            
            // Build the same CSV string as before
            sb.append(String.format("%.3f,", timestamp));
            sb.append(currentState.toString()).append(",");
            sb.append(String.format("%.4f,", getHeight()));
            sb.append(String.format("%.4f,", getCurrentSetpoint()));
            sb.append(String.format("%.4f,", motor1.get()));
            sb.append(String.format("%.2f,", motor1.getOutputCurrent()));
            sb.append(String.format("%.2f,", motor2.getOutputCurrent()));
            sb.append(String.format("%.2f,", motor1.getBusVoltage()));
            sb.append(String.format("%.2f,", motor2.getBusVoltage()));
            sb.append(String.format("%.1f,", motor1.getMotorTemperature()));
            sb.append(String.format("%.1f,", motor2.getMotorTemperature()));
            sb.append(lowerLimit.get()).append(",");
            sb.append(isStalled).append(",");
            sb.append(stallCount).append(",");
            sb.append(isManualControl).append(",");
            sb.append(softLimitsEnabled).append(",");
            sb.append(event);
            
            logWriter.println(sb.toString());
            
            // Periodically flush to ensure data is written
            if (timestamp % 1.0 < 0.02) { // Flush every second approximately
                logWriter.flush();
            }
            
        } catch (Exception e) {
            System.err.println("Failed to log data: " + e.getMessage());
            stopLogging();
            // Attempt to restart logging if it was a temporary error
            if (isLogging) {
                startLogging();
            }
        }
    }
}

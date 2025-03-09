package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase {
  // Hardware
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  
  // Motor IDs
  private static final int LEFT_MOTOR_ID = CoralConstants.leftCoralID;
  private static final int RIGHT_MOTOR_ID = CoralConstants.rightCoralID;
  
  // Current limiting
  private static final int SCL = CoralConstants.SCL; // Smart Current Limit
  private static final int FCL = CoralConstants.FCL; // Free Current Limit

  // Game piece detection thresholds
  private static double CURRENT_THRESHOLD = CoralConstants.currentThreshold; // Default threshold
  private static final double DETECTION_TIME = CoralConstants.detectionTime; // Time current must be above threshold
  
  // Detection tracking
  private boolean gamePresent = false;
  private double highCurrentStartTime = 0;

  // Simple speed control
  private double motorSpeed = 0.0;

  public enum MotorDirection {
    FORWARD,
    REVERSE,
    STOPPED
  }

  private MotorDirection currentDirection = MotorDirection.STOPPED;

  public Coral() {
    // Initialize motors
    leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

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
    // Update motors with current speed
    leftMotor.set(motorSpeed);
    rightMotor.set(0.25 * motorSpeed);
    
    // Update game piece detection
    updateGamePieceDetection();
    
    // Log data
    SmartDashboard.putString("Coral/Coral Direction", currentDirection.toString());
    SmartDashboard.putNumber("Coral/Coral Motor Speed", motorSpeed);
    SmartDashboard.putNumber("Coral/Left Motor Velocity", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Coral/Right Motor Velocity", rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Coral/Left Current Draw", getLeftCurrentDraw());
    SmartDashboard.putNumber("Coral/Right Current Draw", getRightCurrentDraw());
    SmartDashboard.putBoolean("Coral/Game Piece Detected", hasGamePiece());
  }

  /**
   * Updates the game piece detection status based on motor current
   */
  private void updateGamePieceDetection() {
    // We'll use the left motor for detection since it gets full power
    double currentDraw = getLeftCurrentDraw();
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // Only detect when motors are running in forward direction
    if (currentDirection == MotorDirection.FORWARD && motorSpeed > 0.1) {
      if (currentDraw >= CURRENT_THRESHOLD) {
        // If this is the start of high current, record the time
        if (highCurrentStartTime == 0) {
          highCurrentStartTime = currentTime;
        }
        
        // Check if current has been high for the required duration
        if (currentTime - highCurrentStartTime >= DETECTION_TIME) {
          gamePresent = true;
        }
      } else {
        // Reset the timer if current drops below threshold
        highCurrentStartTime = 0;
      }
    } else {
      // Reset detection when not intaking
      highCurrentStartTime = 0;
      // Don't reset gamePresent here - we want it to stay true until we release the game piece
    }
    
    // Reset the game piece detection when we reverse or stop
    if (currentDirection == MotorDirection.REVERSE || currentDirection == MotorDirection.STOPPED) {
      gamePresent = false;
    }
  }

  /**
   * Sets the current threshold for game piece detection
   * @param threshold The current threshold in amps
   */
  public void setCurrentThreshold(double threshold) {
    CURRENT_THRESHOLD = threshold;
  }

  /**
   * @return True if a game piece is detected in the intake
   */
  public boolean hasGamePiece() {
    return gamePresent;
  }

  // Simple control methods
  public void forward(double speed) {
    motorSpeed = Math.abs(speed); // Ensure positive value
    currentDirection = MotorDirection.FORWARD;
  }

  public void reverse(double speed) {
    motorSpeed = -Math.abs(speed); // Ensure negative value
    currentDirection = MotorDirection.REVERSE;
  }

  public void stop() {
    motorSpeed = 0.0;
    currentDirection = MotorDirection.STOPPED;
  }
  
  // Getter methods
  public MotorDirection getCurrentDirection() {
    return currentDirection;
  }
  
  public double getMotorSpeed() {
    return motorSpeed;
  }
  
  public double getLeftCurrentDraw() {
    return leftMotor.getOutputCurrent();
  }
  
  public double getRightCurrentDraw() {
    return rightMotor.getOutputCurrent();
  }
}
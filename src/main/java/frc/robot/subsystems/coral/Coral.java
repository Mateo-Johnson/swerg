package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralAlgaeConstants;

public class Coral extends SubsystemBase {
  // Hardware
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  
  // Current limiting
  private static final int SCL = CoralAlgaeConstants.SCL; // Smart Current Limit
  private static final int FCL = CoralAlgaeConstants.FCL; // Free Current Limit

  // Detection tracking
    private boolean gamePresent = CoralAlgaeConstants.gamePresent;
    private double highCurrentStartTime = CoralAlgaeConstants.highCurrentStartTime;
    private boolean motorStartupIgnore = CoralAlgaeConstants.motorStartupIgnore;
    private double motorStartTime = CoralAlgaeConstants.motorStartTime;
    private static final double ignore = CoralAlgaeConstants.ignoreTime; // 500ms to ignore initial startup current spike
    private static final double debounce = CoralAlgaeConstants.debounce; // 100ms debounce for current detection

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
    leftMotor = CoralAlgaeConstants.leftCoral;
    rightMotor = CoralAlgaeConstants.rightCoral;

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
    
    // Update game piece detection - only if we're intaking
    if (currentDirection == MotorDirection.FORWARD) {
      updateGamePieceDetection();
    }
    
    // Log data
    SmartDashboard.putString("Coral/Coral Direction", currentDirection.toString());
    SmartDashboard.putNumber("Coral/Coral Motor Speed", motorSpeed);
    SmartDashboard.putNumber("Coral/Left Motor Velocity", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Coral/Left Current Draw", getLeftCurrentDraw());
    SmartDashboard.putBoolean("Coral/GP", hasGamePiece());
    SmartDashboard.putBoolean("Coral/Startup Ignore", motorStartupIgnore);
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

    // We'll use the left motor for detection since it gets full power
    double currentDraw = getLeftCurrentDraw();
    double currentTime = Timer.getFPGATimestamp();
    
    // Handle startup current spike ignore logic
    if (motorStartupIgnore && currentDirection != MotorDirection.STOPPED) {
      // If we're ignoring startup and motors are running
      if (motorStartTime == 0) {
        // First time we've seen motors running since stop
        motorStartTime = currentTime;
      } else if (currentTime - motorStartTime > ignore) {
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
        } else if (currentTime - highCurrentStartTime > debounce) { // 100ms debounce
          // We've seen high current for enough time
          gamePresent = true;
        }
      } else {
        // Current is below threshold, reset timing
        highCurrentStartTime = 0;
      }
    }
  }

  /**
   * @return True if a game piece is detected in the intake
   */
  public boolean hasGamePiece() {
    return gamePresent;
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

  // Simple control methods
  public void forward(double speed) {
    if (currentDirection == MotorDirection.STOPPED) {
      // If starting from stopped, reset timers
      motorStartTime = 0;
    }
    motorSpeed = Math.abs(speed); // Ensure positive value
    currentDirection = MotorDirection.FORWARD;
  }

  public void reverse(double speed) {
    if (currentDirection == MotorDirection.STOPPED) {
      // If starting from stopped, reset timers
      motorStartTime = 0;
    }
    motorSpeed = -Math.abs(speed); // Ensure negative value
    currentDirection = MotorDirection.REVERSE;
    
    // When reversing, we're ejecting a game piece, so reset detection
    resetGamePieceDetection();
  }

  public void stop() {
    motorSpeed = 0.0;
    currentDirection = MotorDirection.STOPPED;
    // Do NOT reset gamePresent when stopping - only when ejecting
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
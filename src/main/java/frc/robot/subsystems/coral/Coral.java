package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralConstants;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase {
  // Hardware
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private AnalogInput sonar;
  
  // Motor IDs
  private static final int LEFT_MOTOR_ID = CoralConstants.leftCoralID;
  private static final int RIGHT_MOTOR_ID = CoralConstants.rightCoralID;
  
  // Current limiting
  private static final int SCL = CoralConstants.SCL; // Smart Current Limit
  private static final int FCL = CoralConstants.FCL; // Free Current Limit

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
    sonar = new AnalogInput(4);

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
    rightMotor.set(0.2 * motorSpeed);
    
    // Log data
    SmartDashboard.putString("Coral/Coral Direction", currentDirection.toString());
    SmartDashboard.putNumber("Coral/Coral Motor Speed", motorSpeed);
    SmartDashboard.putNumber("Coral/Left Motor Output", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Coral/Right Motor Output", rightMotor.getAppliedOutput());
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
  
  // GETTER METHODS
  public double getSonarDistance() {
    double voltage = sonar.getVoltage();

    // Adjust scaling
    return voltage = voltage / 0.0098;
  }

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
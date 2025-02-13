package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final AnalogEncoder m_turningAnalogEncoder;

  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;
  
  private final SimpleMotorFeedforward m_drivingFeedforward;

  private double m_chassisAngularOffset = 0;
  private final double m_analogEncoderOffset;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private boolean m_hasBeenInitialized = false;
  private String m_moduleName;

  /**
   * Constructs a MAXSwerveModule with a Thrifty absolute encoder
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int analogPort, double analogOffset, String moduleName) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_moduleName = moduleName;

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    m_turningAnalogEncoder = new AnalogEncoder(analogPort);

    // Configure driving PID (P=1, I=0, D=0)
    m_drivingPIDController = new PIDController(1.0, 0, 0);
    
    // Configure turning PID with the correct gains (P=0.04, I=0, D=0)
    m_turningPIDController = new PIDController(.1, 0, 0);

    // Configure the turning PID controller to be continuous
    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

    // Configure feedforward
    m_drivingFeedforward = new SimpleMotorFeedforward(
        0, // kS
        1 / ModuleConstants.kDriveWheelFreeSpeedRps  // kV
    );

    m_analogEncoderOffset = analogOffset;

    // Apply non-PID configurations
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    // Initialize the relative encoder
    initializeModule();
  }

  private void initializeModule() {
    if (!m_hasBeenInitialized) {
      // Get the absolute position from the analog encoder
      double absolutePosition = getAbsoluteAngle();
      
      // Set the relative encoder to match the absolute position
      m_turningEncoder.setPosition(absolutePosition);
      
      m_hasBeenInitialized = true;
    }
  }

  private double getAbsoluteAngle() {
    double position = m_turningAnalogEncoder.get();
    position *= 2.0 * Math.PI;
    position -= m_analogEncoderOffset;
    
    position %= 2.0 * Math.PI;
    if (position < 0.0) {
        position += 2.0 * Math.PI;
    }

    return position;
  }

  public double getAngle() {
    // Use the relative encoder for normal operation
    return m_turningEncoder.getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getAngle()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Ensure module is initialized
    initializeModule();

    double currentAngleRadians = getAngle();
    Rotation2d currentRotation = new Rotation2d(currentAngleRadians);
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, currentRotation);
    
    // Calculate PID outputs
    double drivingPIDOutput = m_drivingPIDController.calculate(
        m_drivingEncoder.getVelocity(),
        optimizedDesiredState.speedMetersPerSecond
    );

    double drivingFeedforward = m_drivingFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond);
    
    double turningOutput = m_turningPIDController.calculate(
        getAngle(),
        optimizedDesiredState.angle.getRadians()
    );
    
    // Clamp outputs to [-1, 1] range
    double drivingOutput = clamp(drivingPIDOutput + drivingFeedforward, -1.0, 1.0);
    turningOutput = clamp(turningOutput, -1.0, 1.0);
    
    // Apply the outputs to the motors
    m_drivingSpark.set(drivingOutput);
    m_turningSpark.set(turningOutput);
    
    m_desiredState = desiredState;
  }

  private double clamp(double value, double min, double max) {
    return Math.min(max, Math.max(min, value));
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    // Reset using the absolute encoder position
    double absolutePosition = getAbsoluteAngle();
    m_turningEncoder.setPosition(absolutePosition);
    m_hasBeenInitialized = true;
  }

  public void updateSmartDashboard() {
    double currentAngleDegrees = Math.toDegrees(getAngle());
    double desiredAngleDegrees = Math.toDegrees(m_desiredState.angle.getRadians());
    double absoluteAngleDegrees = Math.toDegrees(getAbsoluteAngle());
    
    // Add more detailed debugging info
    SmartDashboard.putNumber(m_moduleName + " Current Angle", currentAngleDegrees);
    SmartDashboard.putNumber(m_moduleName + " Desired Angle", desiredAngleDegrees);
    SmartDashboard.putNumber(m_moduleName + " Absolute Angle", absoluteAngleDegrees);
    SmartDashboard.putNumber(m_moduleName + " Angle Error", desiredAngleDegrees - currentAngleDegrees);
    SmartDashboard.putNumber(m_moduleName + " Applied Turn Output", m_turningSpark.getAppliedOutput());
    SmartDashboard.putNumber(m_moduleName + " Turn PID Output", m_turningPIDController.calculate(getAngle()));
  }
}
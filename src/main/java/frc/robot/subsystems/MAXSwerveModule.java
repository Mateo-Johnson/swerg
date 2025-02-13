package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;


public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final AnalogEncoder m_turningAnalogEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final PIDController m_turningPIDController;

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
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    
    m_turningPIDController = new PIDController(1.0, 0.0, 0.0);
    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

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
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    
    double turningOutput = MathUtil.clamp(m_turningPIDController.calculate(getAngle(), correctedDesiredState.angle.getRadians()), -1.0, 1.0);;
    
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningSpark.set(turningOutput);
    
    m_desiredState = desiredState;
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
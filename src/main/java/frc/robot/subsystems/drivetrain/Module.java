package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.Config;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Module {
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final AnalogEncoder m_turningAnalogEncoder;
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
    private final double m_analogEncoderOffset;
    @SuppressWarnings("unused")
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public Module(int drivingCANId, int turningCANId, int analogPort, double analogOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getEncoder();
        m_turningAnalogEncoder = new AnalogEncoder(analogPort);
        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
        m_analogEncoderOffset = analogOffset;

        // Apply configurations
        m_drivingSpark.configure(Config.Module.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Config.Module.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
                
        // Reset the encoders during initialization
        syncEncoders();
    }

    public double getAngle() {
        double pos = m_turningAnalogEncoder.get();
        double position = pos * (2 * Math.PI);
        position = position - m_analogEncoderOffset;
        return MathUtil.angleModulus(position);
    }

    public double getRawRadians() {
        double pos = m_turningAnalogEncoder.get();
        double position = pos * (2 * Math.PI);
        position = position % (2 * Math.PI);
        if (position > Math.PI) {
            position -= 2 * Math.PI;
        } else if (position < -Math.PI) {
            position += 2 * Math.PI;
        }
        return position;
    }

    public double getRawAngle() {
        double raw = m_turningAnalogEncoder.get();
        return raw;
    }

    private void syncEncoders() {
        // Reset the encoder positions to 0 on initialization
        double angle = getAngle();
        m_turningClosedLoopController.setReference(0, ControlType.kDutyCycle);
        m_turningEncoder.setPosition(0);  // Reset turning encoder to zero
        m_drivingEncoder.setPosition(0); // Reset driving encoder to zero
        double targetAngle = angle;
        m_turningClosedLoopController.setReference(targetAngle, ControlType.kPosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
        
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void resetWheels() {
        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public void resetEncoders() {
        syncEncoders(); // This will reset the encoder values to zero
    }
}

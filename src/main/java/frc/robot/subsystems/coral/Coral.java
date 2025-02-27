package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase {
    // Hardware
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final AnalogInput sonar;
    
    // Motor IDs
    private static final int LEFT_MOTOR_ID = CoralConstants.leftCoralID;
    private static final int RIGHT_MOTOR_ID = CoralConstants.rightCoralID;
    
    // Current limiting
    private static final int SCL = CoralConstants.SCL; // Smart Current Limit
    private static final int FCL = CoralConstants.FCL; // Free Current Limit
    
    // PID Constants - initial values
    private double kP = 0.0005;
    private double kI = 0.000001;
    private double kD = 0.0;
    
    private boolean tuningEnabled = false;
    private double lastP = kP;
    private double lastI = kI;
    private double lastD = kD;
    
    // Speed control
    private double targetVelocity = 0.0;

    public enum MotorDirection {
        FORWARD,
        REVERSE,
        STOPPED
    }
    private MotorDirection currentDirection = MotorDirection.STOPPED;

    public Coral() {
        leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
        sonar = new AnalogInput(4);
        configureMotors();
        initializeSmartDashboard();
    }

    private void configureMotors() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        
        leftConfig
            .smartCurrentLimit(SCL, FCL)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
            
        rightConfig
            .smartCurrentLimit(SCL, FCL)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        leftConfig.encoder
            .positionConversionFactor(8192)
            .velocityConversionFactor(8192);
            
        rightConfig.encoder
            .positionConversionFactor(8192)
            .velocityConversionFactor(8192);

        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD);
            
        rightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void initializeSmartDashboard() {
        SmartDashboard.putNumber("Coral/PID P", kP);
        SmartDashboard.putNumber("Coral/PID I", kI);
        SmartDashboard.putNumber("Coral/PID D", kD);
        SmartDashboard.putBoolean("Coral/PID Tuning Enabled", tuningEnabled);
    }

    @Override
    public void periodic() {
        checkTuningValues();
        leftMotor.getClosedLoopController().setReference(targetVelocity, ControlType.kVelocity);
        rightMotor.getClosedLoopController().setReference(targetVelocity, ControlType.kVelocity);

        SmartDashboard.putString("Coral/Coral Direction", currentDirection.toString());
        SmartDashboard.putNumber("Coral/Target Velocity", targetVelocity);
        SmartDashboard.putNumber("Coral/Left Motor Velocity", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Coral/Right Motor Velocity", rightMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Coral/Left Motor Output", leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Coral/Right Motor Output", rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Coral/Velocity Error", 
            leftMotor.getEncoder().getVelocity() - rightMotor.getEncoder().getVelocity());
    }

    private void checkTuningValues() {
        boolean newTuningEnabled = SmartDashboard.getBoolean("Coral/PID Tuning Enabled", false);
        if (newTuningEnabled != tuningEnabled) {
            tuningEnabled = newTuningEnabled;
            SmartDashboard.putBoolean("Coral/PID Tuning Enabled", tuningEnabled);
        }

        if (tuningEnabled) {
            double newP = SmartDashboard.getNumber("Coral/PID P", kP);
            double newI = SmartDashboard.getNumber("Coral/PID I", kI);
            double newD = SmartDashboard.getNumber("Coral/PID D", kD);
            
            boolean valuesChanged = (newP != lastP) || (newI != lastI) || (newD != lastD);
            
            if (valuesChanged) {
                kP = newP;
                kI = newI;
                kD = newD;
                lastP = kP;
                lastI = kI;
                lastD = kD;
                
                updatePIDControllers();
                System.out.println("PID values updated - P: " + kP + ", I: " + kI + ", D: " + kD);
            }
        }
    }

    private void updatePIDControllers() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(kP, kI, kD);
        
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void forward(double speed) {
        targetVelocity = Math.abs(speed) * 5600;
        currentDirection = MotorDirection.FORWARD;
    }

    public void reverse(double speed) {
        targetVelocity = -Math.abs(speed) * 5600;
        currentDirection = MotorDirection.REVERSE;
    }

    public void stop() {
        targetVelocity = 0.0;
        currentDirection = MotorDirection.STOPPED;
    }

    public double getSonarDistance() {
        double voltage = sonar.getVoltage();
        return voltage / 0.0098;
    }

    public MotorDirection getCurrentDirection() {
        return currentDirection;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightMotor.getEncoder().getVelocity();
    }

    public double getLeftCurrentDraw() {
        return leftMotor.getOutputCurrent();
    }

    public double getRightCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }

    public void enablePIDTuning(boolean enable) {
        tuningEnabled = enable;
        SmartDashboard.putBoolean("Coral/PID Tuning Enabled", tuningEnabled);
    }

    public void updatePIDValues(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        lastP = kP;
        lastI = kI;
        lastD = kD;
        SmartDashboard.putNumber("Coral/PID P", kP);
        SmartDashboard.putNumber("Coral/PID I", kI);
        SmartDashboard.putNumber("Coral/PID D", kD);
        updatePIDControllers();
    }
}
package frc.robot.subsystems;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public final class Config {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
            double turningFactor = 2 * Math.PI;
            
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // M
                    .velocityConversionFactor(drivingFactor / 60.0); // M/S
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            
//TURNING CONFIG
turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

turningConfig.encoder
        //CONVERT MOTOR ROTATIONS TO RADIANS
        .positionConversionFactor(2 * Math.PI) //FULL ROTATION (360°)
        .velocityConversionFactor(turningFactor / 60.0);

turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1, 0.0, 0.00)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, (2*Math.PI));
        }
    }
}
package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.commands.AlignY;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LimelightLib;

public class Drivetrain extends SubsystemBase {

  // Create a field object to display the path on the dashboard
  private Field2d field = new Field2d();
 
  // Create the modules
  // Front Left
  private final Module m_frontLeft = new Module(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftEncoder,
      2.88);

  // Front Right
  private final Module m_frontRight = new Module(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightEncoder,
      5.14);

  // Rear Left
  private final Module m_rearLeft = new Module(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftEncoder,
      2.8);

  // Rear Right
  private final Module m_rearRight = new Module(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightEncoder,
      5.7);

  // PID controller for heading
  private PIDController headingCorrector = new PIDController(0.032, 0, 0.0015);

  // Create gyro (NavX)
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Odometry for tracking pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // Pose estimator for tracking pose again
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(-m_gyro.getAngle()),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d()  // Initial pose
);

  public Drivetrain() {

    // Pathplanner things
    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetOdometry, 
        this::getRobotRelativeSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          new PIDConstants(0.0001, 0.0, 0.0), // Translation PID constants
          new PIDConstants(0.0001, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {

    Pose2d visionPose = LimelightLib.getBotPose2d_wpiBlue("limelight");
    double latency = LimelightLib.getLatency_Pipeline("limelight") / 1000.0; // Convert ms to seconds
    updateWithVision(visionPose, latency);

    // Update the field pose
    field.setRobotPose(getPose());

    // Update the odometry
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update the pose estimator
    m_poseEstimator.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Positioning dashboard
    SmartDashboard.putNumber("POSITION/Heading", getHeading());
    SmartDashboard.putNumber("POSITION/current x", getPose().getX());
    SmartDashboard.putNumber("POSITION/current y", getPose().getY());

    // Drivetrain module angles (maybe module widget?)
    SmartDashboard.putNumber("DT/FL", m_frontLeft.getAngleFull());
    SmartDashboard.putNumber("DT/FR", m_frontRight.getAngleFull());
    SmartDashboard.putNumber("DT/RL", m_rearLeft.getAngleFull());
    SmartDashboard.putNumber("DT/RR", m_rearRight.getAngleFull());
    SmartDashboard.putBoolean("ALIGNING", AlignY.isAligning);


  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();
    // Invert coordinates to match the transformation in resetOdometry
    return new Pose2d(
        -estimatedPose.getX(),
        -estimatedPose.getY(),
        estimatedPose.getRotation()
    );
  }

  /**
   * Returns the current robot-relative chassis speeds
   * 
   * @return ChassisSpeeds object containing the robot's velocity components
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the current states of each swerve module
    SwerveModuleState frontLeftState = m_frontLeft.getState();
    SwerveModuleState frontRightState = m_frontRight.getState();
    SwerveModuleState rearLeftState = m_rearLeft.getState();
    SwerveModuleState rearRightState = m_rearRight.getState();

    // Convert the swerve module states to chassis speeds
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  /**
   * Drives the robot with the specified robot-relative chassis speeds.
   * 
   * @param speeds The desired robot-relative chassis speeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
      // 
      ChassisSpeeds invertedSpeeds = new ChassisSpeeds(
          -speeds.vxMetersPerSecond,
          -speeds.vyMetersPerSecond,
          -speeds.omegaRadiansPerSecond
      );
      
      // Convert the inverted speeds to swerve module states
      SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(invertedSpeeds);

      // Desaturate the wheel speeds
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

      // Set the desired state for each swerve module
      setModuleStates(moduleStates);
  }

  public void resetWheels() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

      // Invert the coordinates to match the transformation in getPose()
      Pose2d invertedPose = new Pose2d(
          -pose.getX(),
          -pose.getY(),
          pose.getRotation()
      );
      
      // Reset the pose estimator
      m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        invertedPose
    );

    // Reset the odometry
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        invertedPose
    );
}

  /**
   * Method to drive the robot while holding an angle.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param targetHeading The heading to hold the drivetrain to
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveHeadingLocked(double xSpeed, double ySpeed, double targetHeading, boolean fieldRelative) {
      // Calculate rotation correction to maintain heading
      double currentHeading = getHeading();
      double rotationCorrection = headingCorrector.calculate(currentHeading, targetHeading);
     
      // Clamp the rotation correction between -1 and 1
      rotationCorrection = Math.max(-1, Math.min(1, rotationCorrection));
     
      // Use the normal drive method with the correction
      drive(xSpeed, ySpeed, rotationCorrection, fieldRelative);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert commanded speeds into correct for drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // Create ChassisSpeeds object with correct coordinate system
    var chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // Change it to module states
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    // Set states for each module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    // Get the angle from the gyro
    double angle = -m_gyro.getAngle();
    
    // Wrap to -180 to 180 range
    angle = MathUtil.inputModulus(angle, -180.0, 180.0);
    
    return angle;
}

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void updateWithVision(Pose2d visionPose, double latencySeconds) {
    // Add the vision measurement to the pose estimator
    m_poseEstimator.addVisionMeasurement(
        visionPose,
        latencySeconds
    );
}
}
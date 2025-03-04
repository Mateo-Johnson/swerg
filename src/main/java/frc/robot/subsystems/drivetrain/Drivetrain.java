package frc.robot.subsystems.drivetrain;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;


public class Drivetrain extends SubsystemBase {
 
  //CREATE MODULES


  //FRONT LEFT
  private final Module m_frontLeft = new Module(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftEncoder,
      2.9287459531721316);


  //FRONT RIGHT
  private final Module m_frontRight = new Module(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightEncoder,
      5.110521631358809);


  //REAR LEFT
  private final Module m_rearLeft = new Module(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftEncoder,
      2.772308406337558);


  //REAR RIGHT
  private final Module m_rearRight = new Module(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightEncoder,
      5.723793939068865);

  //PID CONTROLLERS
  private PIDController headingCorrector = new PIDController(0.1, 0, 0.01);

  //CREATE GYRO (NAVX)
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  //ODOMETRY FOR TRACKING ROBOT
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });




  public Drivetrain() {
    // zeroHeading();


  }


  @Override
  public void periodic() {
    //UPDATE THE ODOMETRY IN PERIODIC
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });




    SmartDashboard.putNumber("angles2/FL", m_frontLeft.getAngleFull());
    SmartDashboard.putNumber("angles2/RL", m_rearLeft.getAngleFull());
    SmartDashboard.putNumber("angles2/FR", m_frontRight.getAngleFull());
    SmartDashboard.putNumber("angles2/RR", m_rearRight.getAngleFull());


    // SmartDashboard.putNumber("angles1/FL", m_frontLeft.m_turningEncoder.getPosition());
    // SmartDashboard.putNumber("angles1/RL", m_rearLeft.m_turningEncoder.getPosition());
    // SmartDashboard.putNumber("angles1/FR", m_frontRight.m_turningEncoder.getPosition());
    // SmartDashboard.putNumber("angles1/RR", m_rearRight.m_turningEncoder.getPosition());


    // SmartDashboard.putNumber("commanded/FL", m_frontLeft.getDesiredState().angle.getRadians());
    // SmartDashboard.putNumber("commanded/RL", m_rearLeft.getDesiredState().angle.getRadians());
    // SmartDashboard.putNumber("commanded/FR", m_frontRight.getDesiredState().angle.getRadians());
    // SmartDashboard.putNumber("commanded/RR", m_rearRight.getDesiredState().angle.getRadians());
    double heading = getHeading();
    SmartDashboard.putNumber("Heading", heading);
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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
    // CONVERT COMMANDED SPEEDS INTO CORRECT FOR DRIVETRAIN
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;


    // CREATE CHASSISPEEDS OBJECT WITH CORRECT COORD SYSTEM
    var chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);


    // CHANGE TO MODULE STATES
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);


    // DESATURATE WHEEL SPEEDS
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);


    // SET STATES FOR EACH MODULE
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
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}

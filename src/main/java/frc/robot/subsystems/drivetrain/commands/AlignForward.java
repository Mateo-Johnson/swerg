// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignForward extends Command {
      private final Drivetrain m_drivetrain;
    private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 
  /** Creates a new Forward. */
  public AlignForward(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;

    // Add requirements to ensure command interacts with the swerve drive subsystem
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double targetOffsetY = LimelightLib.getTargetPose3d_CameraSpace("limelight-front").getZ();

      double yPidOutput = yPID.calculate(targetOffsetY, 0.32);
      m_drivetrain.drive(yPidOutput, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

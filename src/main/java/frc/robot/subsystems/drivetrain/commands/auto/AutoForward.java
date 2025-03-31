// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drivetrain.commands.auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoForward extends Command {
      private final Drivetrain m_drivetrain;
  /** Creates a new Forward. */
  public AutoForward(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    // Add requirements to ensure command interacts with the swerve drive subsystem
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drivetrain.drive(-0.1, 0, 0, false);
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
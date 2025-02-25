package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class RunCoralIntake extends Command {
  private final Coral m_coral;
  private final double intakeSpeed = 0.4; // Adjust this value as needed
  
  public RunCoralIntake(Coral subsystem) {
    m_coral = subsystem;
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {
    // Start the intake when the command begins
    m_coral.forward(intakeSpeed);
  }
  
  @Override
  public void execute() {
    // Command continuously runs the intake
    // Nothing additional needed here as the motor continues at the set speed
  }
  
  @Override
  public void end(boolean interrupted) {
    // Stop the intake when the command ends
    m_coral.stop();
  }
  
  @Override
  public boolean isFinished() {
    // This will run until interrupted (when the button is released)
    return false;
  }
}
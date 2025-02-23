package frc.robot.subsystems.vision.commands.game;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.commands.general.AlignRotate;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAlgae extends Command {
  /** Creates a new AlignReefLeft. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private AlignRotate alignRotateCommand;  // Declare AlignRotate command

  public AlignAlgae(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
    this.m_vision = vision;
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double tX = m_vision.getTX();
    double setpoint = 0;
    
    // Create the AlignRotate command
    alignRotateCommand = new AlignRotate(setpoint, 0.1, tX, m_drivetrain);
    
    // Schedule the AlignRotate command to run
    alignRotateCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No need to create a new AlignRotate here
    // The AlignRotate command is already running and will end when it reaches the setpoint.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Optionally cancel the AlignRotate command if needed
    if (alignRotateCommand != null) {
      alignRotateCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true when the AlignRotate command is finished
    return alignRotateCommand != null && alignRotateCommand.isFinished();
  }
}

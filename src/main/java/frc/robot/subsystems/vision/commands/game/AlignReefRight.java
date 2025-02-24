package frc.robot.subsystems.vision.commands.game;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Constants.PIDConstants;

public class AlignReefRight extends Command {
  /** Creates a new AlignReefRight. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final PIDController reefPID = PIDConstants.xPID;

  public AlignReefRight(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
    this.m_vision = vision;
    addRequirements(drivetrain, vision);

    reefPID.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tX = m_vision.getTX();
    double setpoint = 5;
    double output = reefPID.calculate(tX, setpoint);
    output = Math.max(-1, Math.min(1, output));

    m_drivetrain.drive(0, output, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reefPID.atSetpoint();
  }
}
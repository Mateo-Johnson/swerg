package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

/**
 * Command to move the Algae subsystem to a specific position over a specified time
 */
public class MoveOverTime extends Command {
    private final Algae m_algae;
    private final double m_position;
    private final double m_duration;
    
    /**
     * Creates a new MoveOverTime command
     * 
     * @param algae The Algae subsystem
     * @param position The target position to move to
     * @param duration The duration in seconds to complete the move
     */
    public MoveOverTime(Algae algae, double position, double duration) {
        m_algae = algae;
        m_position = position;
        m_duration = duration;
        addRequirements(algae);
    }
    
    @Override
    public void initialize() {
        m_algae.moveToPositionOverTime(m_position, m_duration);
    }
    
    @Override
    public boolean isFinished() {
        return m_algae.getState() == Algae.STATES.AT_SETPOINT;
    }
}
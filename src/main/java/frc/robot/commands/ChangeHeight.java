package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ChangeHeight extends Command {
    private final Elevator m_elevator;
    private double m_setpoint;

    public ChangeHeight(Elevator elevator, double setpoint) {
        m_elevator = elevator;
        m_setpoint = setpoint;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.changeSetpoint(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_setpoint,m_elevator.elevatorEncoder.getPosition(),10);
    }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ChangeAngle extends Command {
    private final Arm m_arm;
    private double m_setpoint;

    public ChangeAngle(Arm arm, double setpoint) {
        m_arm = arm;
        m_setpoint = setpoint;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.changeSetpoint(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_setpoint,m_arm.armEncoder.getPosition(),3);
    }
}

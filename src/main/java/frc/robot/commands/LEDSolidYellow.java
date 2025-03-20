package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class LEDSolidYellow extends Command {
    private final LED m_led;
    public LEDSolidYellow(LED led){
        m_led = led;

        addRequirements(m_led);
    }

    @Override
    public void initialize(){
        m_led.SolidYellow();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;

public class RobotCentricDrive extends Command{
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    LED m_led;

    public RobotCentricDrive(DriveSubsystem drive, LED led, CommandXboxController controller){
        m_drive = drive;
        m_controller = controller;
        m_led = led;

        addRequirements(m_drive, m_led);
    }

    @Override
    public void execute(){
        m_drive.drive(
                    -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(4), OIConstants.kDriveDeadband),
                false);
        m_led.SolidPink();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
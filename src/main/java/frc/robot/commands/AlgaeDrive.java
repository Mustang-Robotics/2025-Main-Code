package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AlgaeDrive extends Command{
    DriveSubsystem m_drive;
    double m_speed;

    public AlgaeDrive(DriveSubsystem drive, double speed){
        m_drive = drive;
        m_speed = speed;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        m_drive.drive(-0, -m_speed, 0, false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
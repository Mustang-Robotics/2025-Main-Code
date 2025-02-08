// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb_Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb_Command extends Command {
  /** Creates a new Climb_Command. */
Climb_Motor m_Climb_Motor;
double m_climbSpeed;

  public Climb_Command(Climb_Motor climb_Motor, double climbSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    climb_Motor = m_Climb_Motor;
    m_climbSpeed = climbSpeed;
    addRequirements(m_Climb_Motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climb_Motor.setClimbMotorSpeed(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climb_Motor.setClimbMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

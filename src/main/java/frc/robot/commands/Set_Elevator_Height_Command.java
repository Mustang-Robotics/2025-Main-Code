// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Set_Elevator_Height_Command extends Command {
  /** Creates a new Set_Elevator_Height_Command. */
  Elevator m_Elevator;
  double m_height;

  double tolerance;

  public Set_Elevator_Height_Command(Elevator elevator, double height) {
    m_Elevator = elevator;
    m_height = height;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return m_Elevator.getElevatorHeight() >= m_height+tolerance || m_Elevator.getElevatorHeight() <= m_height-tolerance;
   return true;
  }
}

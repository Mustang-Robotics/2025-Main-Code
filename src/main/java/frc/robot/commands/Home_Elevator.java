// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Home_Elevator extends Command {
  /** Creates a new Home_Elevator. */

  Elevator m_Elevator;
  public Home_Elevator(Elevator elevator) {
   m_Elevator=elevator;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_Elevator.setElevatorSpeed(.1);
    new WaitCommand(.5);
   // m_Elevator.setElevatorSpeed(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // m_Elevator.setElevatorSpeed(0);
   // return m_Elevator.getBottomLimitSwitch();
   return true;
  }
}

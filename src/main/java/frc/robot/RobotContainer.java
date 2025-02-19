// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.CoralStation;
import frc.robot.subsystems.Intake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator();
  private final Climb m_climb = new Climb();
  private final Intake m_intake = new Intake();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> m_chooser;  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        
        m_climb.setDefaultCommand(
          new RunCommand(
            () -> m_climb.setClimbSpeed(m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis()),
          m_climb));
//PathPlannerPath A = PathPlannerPath.fromPathFile("A");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controller Configs
    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(
            () -> m_intake.setIntakeSpeed(0.2),
            m_intake));
    //Operator Controller Configs
    m_operatorController.pov(270)
      .onTrue(new CoralStation(m_elevator, m_arm));
    m_operatorController.pov(180)
      .onTrue(new RunCommand(
      () -> m_elevator.changeSetpoint(ElevatorHeights.kL2Height)));
    m_operatorController.pov(90)
      .onTrue(new L3(m_elevator, m_arm));
    m_operatorController.pov(0)
      .onTrue(new L4(m_elevator, m_arm));


  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return m_chooser.getSelected();
  }  
}

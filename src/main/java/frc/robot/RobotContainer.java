// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;




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
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  GenericHID m_selectorController = new GenericHID(OIConstants.kSelectorControllerPort);
  private final SendableChooser<Command> m_chooser;  

  public Command apathCommand;
  public Command bpathCommand;
  public Command cpathCommand;
  public Command dpathCommand;
  public Command epathCommand;
  public Command fpathCommand;
  public Command gpathCommand;
  public Command hpathCommand;
  public Command ipathCommand;
  public Command jpathCommand;
  public Command kpathCommand;
  public Command lpathCommand;

  public Command m_pathSelected = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
      * @throws ParseException 
      * @throws IOException 
      * @throws FileVersionException 
      */
     public RobotContainer() throws FileVersionException, IOException, ParseException {
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
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        
        m_climb.setDefaultCommand(
          new RunCommand(
            () -> m_climb.setClimbSpeed(m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis()),
          m_climb));
        buildPaths();
        pathSelector();
        }

          private void buildPaths()throws FileVersionException, IOException, ParseException{

            PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));

          //Added throws FileVersionException, IOException, ParseException to not get the error messages.
            PathPlannerPath A = PathPlannerPath.fromPathFile("B");
            PathPlannerPath B = PathPlannerPath.fromPathFile("B");
            PathPlannerPath C = PathPlannerPath.fromPathFile("C");  
            PathPlannerPath D = PathPlannerPath.fromPathFile("D");
            PathPlannerPath E = PathPlannerPath.fromPathFile("E");
            PathPlannerPath F = PathPlannerPath.fromPathFile("F");
            PathPlannerPath G = PathPlannerPath.fromPathFile("G");
            PathPlannerPath H = PathPlannerPath.fromPathFile("H");
            PathPlannerPath I = PathPlannerPath.fromPathFile("I");
            PathPlannerPath J = PathPlannerPath.fromPathFile("J");
            PathPlannerPath K = PathPlannerPath.fromPathFile("K");
            PathPlannerPath L = PathPlannerPath.fromPathFile("L");

            
            apathCommand = AutoBuilder.pathfindThenFollowPath(A, constraints);
            bpathCommand = AutoBuilder.pathfindThenFollowPath(B, constraints);
            cpathCommand = AutoBuilder.pathfindThenFollowPath(C, constraints);
            dpathCommand = AutoBuilder.pathfindThenFollowPath(D, constraints);
            epathCommand = AutoBuilder.pathfindThenFollowPath(E, constraints);
            fpathCommand = AutoBuilder.pathfindThenFollowPath(F, constraints);
            gpathCommand = AutoBuilder.pathfindThenFollowPath(G, constraints);
            hpathCommand = AutoBuilder.pathfindThenFollowPath(H, constraints);
            ipathCommand = AutoBuilder.pathfindThenFollowPath(I, constraints);
            jpathCommand = AutoBuilder.pathfindThenFollowPath(J, constraints);
            kpathCommand = AutoBuilder.pathfindThenFollowPath(K, constraints);
            lpathCommand = AutoBuilder.pathfindThenFollowPath(L, constraints);
          }


private Command pathSelector() {
           
if (m_selectorController.getRawButton(1)) {
  return m_pathSelected = apathCommand;
} else if (m_selectorController.getRawButton(2)) {
  return m_pathSelected = bpathCommand;
} else if (m_selectorController.getRawButton(3)) {
  return m_pathSelected = cpathCommand;
} else if (m_selectorController.getRawButton(4)) {
  return m_pathSelected = dpathCommand;
} else if (m_selectorController.getRawButton(5)) {
  return m_pathSelected = epathCommand;
}else if (m_selectorController.getRawButton(6)) {
  return m_pathSelected = fpathCommand;
} else if (m_selectorController.getRawButton(7)) {
  return m_pathSelected = gpathCommand;
} else if (m_selectorController.getRawButton(8)) {
  return m_pathSelected = hpathCommand;
} else if (m_selectorController.getRawButton(9)) {
return m_pathSelected = ipathCommand;
} else if (m_selectorController.getRawButton(10)) {
  return m_pathSelected = jpathCommand;
} else if (m_selectorController.getRawButton(11)) {
  return m_pathSelected = kpathCommand;
} else if (m_selectorController.getRawButton(12)) {
return m_pathSelected = lpathCommand;
} else {
  return m_pathSelected = null;
}

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
    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  
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

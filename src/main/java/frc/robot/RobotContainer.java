// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralStation;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Out_take;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kSelectorControllerPort);
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
  public double m_ElevatorHeightSelectorCommand = 0;

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
                -MathUtil.applyDeadband(m_driverController.getLeftY()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*m_elevator.SpeedAdjustment, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        
        m_climb.setDefaultCommand(
          new RunCommand(
            () -> m_climb.setClimbSpeed(m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis()),
          m_climb));
        buildPaths();
        pathSelector();
        elevatorHeightCommand();

        m_driverController.a().onTrue(new InstantCommand(() -> m_pathSelected.schedule()));
        }

          private void buildPaths() throws FileVersionException, IOException, ParseException {

            PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
           
          //Added throws FileVersionException, IOException, ParseException to not get the error messages.
            PathPlannerPath A = PathPlannerPath.fromPathFile("A");
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

            
            GenericEntry Alliance_Collor_Blue = Shuffleboard.getTab("General").add("Alliance Color Blue", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
            Boolean m_Alliance_Collor_Blue = Alliance_Collor_Blue.getBoolean(true);


           
            if (m_Alliance_Collor_Blue == false){
              A.flipPath();
              B.flipPath();
              C.flipPath();
              D.flipPath();
              E.flipPath();
              F.flipPath();
              G.flipPath();
              H.flipPath();
              I.flipPath();
              J.flipPath();
              K.flipPath();
              L.flipPath();
              
             }
            
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
       
          GenericEntry pathSelected = Shuffleboard.getTab("General").add("Path Selected", "A").getEntry();
          String path = pathSelected.getString("A");

private Command pathSelector() {
           
if (m_operatorController.getRawAxis(4)> 0.5) {
  path = "A";
  return m_pathSelected = apathCommand;
} else if (m_operatorController.getRawButton(3)) {
  path = "B";
  return m_pathSelected = bpathCommand;
} else if (m_operatorController.getRawButton(1)) {
  path = "C";
  return m_pathSelected = cpathCommand;
} else if (m_operatorController.getRawButton(2)) {
  path = "D";
  return m_pathSelected = dpathCommand;
} else if (m_operatorController.getRawButton(4)) {
  path = "E";
  return m_pathSelected = epathCommand;
}else if (m_operatorController.getRawButton(6)) {
  path = "F";
  return m_pathSelected = fpathCommand;
} else if (m_operatorController.getRawButton(8)) {
  path = "G";
  return m_pathSelected = gpathCommand;
} else if (m_operatorController.getRawButton(7)) {
  path = "H";
  return m_pathSelected = hpathCommand;
} else if (m_operatorController.getRawButton(5)) {
  path = "I";
return m_pathSelected = ipathCommand;
} else if (m_operatorController.getRawButton(9)) {
  path = "J";
  return m_pathSelected = jpathCommand;
} else if (m_operatorController.getRawAxis(3)>.5) {
  path = "K";
  return m_pathSelected = kpathCommand;
} else if (m_operatorController.getRawButton(10)) {
  path = "L";
return m_pathSelected = lpathCommand;
} else {
  path = "null";
  return m_pathSelected = null;
}

 }

 public double elevatorHeightCommand(){
  if (m_operatorController.getPOV() == 0){
   return m_ElevatorHeightSelectorCommand = 1 /*ElevatorHeights.kL1Height*/;
  }else 
  if (m_operatorController.getPOV() ==180) {
    return m_ElevatorHeightSelectorCommand = 2 /*ElevatorHeights.kL2Height*/;
  }else
  if (m_operatorController.getPOV() == 90) {
    return m_ElevatorHeightSelectorCommand = 3/*ElevatorHeight.kL3Height*/;
  }else
  if (m_operatorController.getPOV() == 270) {
    return m_ElevatorHeightSelectorCommand = 4 /*ElevatorHeights.kL4Height*/;
  }else {
    return m_ElevatorHeightSelectorCommand = 0;
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
    //Driver Controller Configs
    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.rightBumper()
        .whileTrue(new StartEndCommand(
          () -> m_intake.setIntakeSpeed(.2),
          () -> m_intake.setIntakeSpeed(0)));
    
    //Operator Controller Configs
    m_operatorController.pov(0)
      .onTrue(new L4(m_elevator, m_arm));
    m_operatorController.pov(270)
      .onTrue(new CoralStation(m_elevator, m_arm));
     /*m_operatorController.pov(0)
      .onTrue(new RunCommand(
      () -> m_elevator.changeSetpoint(ElevatorHeights.kL1Height)));
    m_operatorController.pov(180)
      .onTrue(new RunCommand(
      () -> m_elevator.changeSetpoint(ElevatorHeights.kL2Height)));
    m_operatorController.pov(90)
      .onTrue(new L3(m_elevator, m_arm));

  
  }

  /**
      .onTrue(new RunCommand(
      () -> m_elevator.changeSetpoint(ElevatorHeights.kL3Height)));
    m_operatorController.pov(0)
      .whileTrue(new RunCommand(
      () -> m_elevator.changeSetpoint(ElevatorHeights.kL4Height)));

      if (m_operatorController.getPOV() == 0){
        new Runnable(()-> m_elevator.changeSetpoint(ElevatorHeights.kL1Height)) {
          
        };
      }*/

      //Lets driver choose when to enable the preselected path and then at point Moves elevator up and then scores
      m_driverController.a().onTrue(m_pathSelected.andThen(()-> m_elevator.moveToSetpoint(m_ElevatorHeightSelectorCommand)).andThen(new Out_take(new Intake())));
      
      
        
      
      
      
        
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

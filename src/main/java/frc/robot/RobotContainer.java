// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;



<<<<<<< HEAD
=======
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.L2;
import frc.robot.commands.CoralStation;
import frc.robot.commands.FollowThenScore;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
>>>>>>> a2f230b (Basically Done)

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
<<<<<<< HEAD
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  public Command pathACommand;
  public Command pathBCommand;
  public Command pathCCommand;
  public Command pathDCommand;
  public Command pathECommand;
  public Command pathFCommand;
  public Command pathGCommand;
  public Command pathHCommand;
  public Command pathICommand;
  public Command pathJCommand;
  public Command pathKCommand;
  public Command pathLCommand;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kSelectorControllerPort);
  private final SendableChooser<Command> m_chooser;  

   // The container for the robot. Contains subsystems, OI devices, and commands.
=======
  public final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();
  private final LED m_led = new LED();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> m_chooser;
  private Command ApathCommand;
  private Command BpathCommand;
  private Command CpathCommand;
  private Command DpathCommand;
  private Command EpathCommand;
  private Command FpathCommand;
  private Command GpathCommand;
  private Command HpathCommand;
  private Command IpathCommand;
  private Command JpathCommand;
  private Command KpathCommand;
  private Command LpathCommand;
  private PathPlannerPath A_path;
  private PathPlannerPath B_path;
  private PathPlannerPath C_path;
  private PathPlannerPath D_path;
  private PathPlannerPath E_path;
  private PathPlannerPath F_path;
  private PathPlannerPath G_path;
  private PathPlannerPath H_path;
  private PathPlannerPath I_path;
  private PathPlannerPath J_path;
  private PathPlannerPath K_path;
  private PathPlannerPath L_path;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
>>>>>>> a2f230b (Basically Done)
      
     public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_led.SolidGreen();
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
             
     }
GenericEntry selectedPath = Shuffleboard.getTab("Auto").add("Selected Path", "").withWidget(BuiltInWidgets.kTextView).getEntry();
Command m_selectedPath;

PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));

public void buildpaths() {
    try {
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
    } catch (FileVersionException | IOException | ParseException e) {
    
      e.printStackTrace();
    }

    pathACommand = AutoBuilder.buildAuto("A");
    pathBCommand = AutoBuilder.buildAuto("B");
    pathCCommand = AutoBuilder.buildAuto("C");
    pathDCommand = AutoBuilder.buildAuto("D");
    pathECommand = AutoBuilder.buildAuto("E");
    pathFCommand = AutoBuilder.buildAuto("F");
    pathGCommand = AutoBuilder.buildAuto("G");
    pathHCommand = AutoBuilder.buildAuto("H");
    pathICommand = AutoBuilder.buildAuto("I");
    pathJCommand = AutoBuilder.buildAuto("J");
    pathKCommand = AutoBuilder.buildAuto("K");
    pathLCommand = AutoBuilder.buildAuto("L");
  
 
}

public Command selectpath() {
if ( m_operatorController.a().getAsBoolean()) {
  return m_selectedPath = pathACommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathBCommand;
} else if ( m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathCCommand;
} else if (m_operatorController.a().getAsBoolean()) {
  return m_selectedPath = pathDCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathECommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathFCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathGCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathHCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathICommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathJCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathKCommand;
} else if (m_operatorController.a().getAsBoolean()){
  return m_selectedPath = pathLCommand;
} else {
  return m_selectedPath = null;
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
<<<<<<< HEAD
    m_driverController.rightBumper()
        .whileTrue(new StartEndCommand(
          () -> m_intake.setIntakeSpeed(.2),
          () -> m_intake.setIntakeSpeed(0)));
    
    //Operator Controller Configs
    m_operatorController.pov(0)
      .onTrue(new L4(m_elevator, m_arm));
    m_operatorController.pov(270)
      .onTrue(new CoralStation(m_elevator, m_arm));
=======
    m_driverController.b()
        .whileTrue(new StartEndCommand(() -> m_intake.setIntakeSpeed(.2), () -> m_intake.setIntakeSpeed(0), m_intake));
    //Operator Controller Configs
    m_driverController.pov(270)
      .onTrue(new CoralStation(m_elevator, m_arm, m_intake, m_led));
    m_driverController.pov(90)
      .onTrue(new L2(m_elevator, m_arm));
    m_driverController.leftBumper()
      .onTrue(new L3(m_elevator, m_arm));
    m_driverController.rightBumper()
      .onTrue(new L4(m_elevator, m_arm));

    m_operatorController.pov(270)
    .onTrue(new RemoveAlgae(m_robotDrive, m_arm, m_intake));
    m_operatorController.pov(180)
    .onTrue(new L2(m_elevator, m_arm));
    m_operatorController.pov(90)
    .onTrue(new L3(m_elevator, m_arm));
    m_operatorController.pov(0)
    .onTrue(new L4(m_elevator, m_arm));

    m_operatorController.rightTrigger(.9).toggleOnTrue(new FollowThenScore(ApathCommand, m_intake, m_led));
    m_operatorController.x().toggleOnTrue(new FollowThenScore(BpathCommand, m_intake, m_led));
    m_operatorController.a().toggleOnTrue(new FollowThenScore(CpathCommand, m_intake, m_led));
    m_operatorController.b().toggleOnTrue(new FollowThenScore(DpathCommand, m_intake, m_led));
    m_operatorController.y().toggleOnTrue(new FollowThenScore(EpathCommand, m_intake, m_led));
    m_operatorController.rightBumper().toggleOnTrue(new FollowThenScore(FpathCommand, m_intake, m_led));
    m_operatorController.start().toggleOnTrue(new FollowThenScore(GpathCommand, m_intake, m_led));
    m_operatorController.back().toggleOnTrue(new FollowThenScore(HpathCommand, m_intake, m_led));
    m_operatorController.leftBumper().toggleOnTrue(new FollowThenScore(IpathCommand, m_intake, m_led));
    m_operatorController.leftStick().toggleOnTrue(new FollowThenScore(JpathCommand, m_intake, m_led));
    m_operatorController.leftTrigger(.9).toggleOnTrue(new FollowThenScore(KpathCommand, m_intake, m_led));
    m_operatorController.rightStick().toggleOnTrue(new FollowThenScore(LpathCommand, m_intake, m_led));


  
>>>>>>> a2f230b (Basically Done)
  }
   /* Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return m_chooser.getSelected();
  }  
}

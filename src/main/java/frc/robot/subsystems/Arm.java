package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Arm extends SubsystemBase {
  //Creates a SparkMax motor controller for the arm & an encoder for the arm
  private final SparkMax m_arm = new SparkMax(9, MotorType.kBrushless);
  private SparkClosedLoopController armClosedLoopController = m_arm.getClosedLoopController();
  public final AbsoluteEncoder armEncoder = m_arm.getAbsoluteEncoder();
  public double armTarget = 50;

  //Creates a feedforward controller for the arm & a PID controller for the arm
  //private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  //private ProfiledPIDController armController = new ProfiledPIDController (0, 0,0,  new TrapezoidProfile.Constraints(0, 0));
  
  //Sets the arm encoder distance per pulse & sets the arm height to 0
  public Arm() {

    m_arm.configure(
      Configs.Arm.armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

  }

  public void changeSetpoint(double setpoint) {
    armTarget = setpoint;
  }

  private void moveToSetpoint() {
    armClosedLoopController.setReference(
      armTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.2);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }
}


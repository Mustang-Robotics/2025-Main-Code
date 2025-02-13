package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  //Creates a SparkMax motor controller for the arm & an encoder for the arm
  //private final SparkMax m_arm = new SparkMax(0, MotorType.kBrushless);
  //private final Encoder m_encoder_arm = new Encoder(2, 3);

  //Creates a feedforward controller for the arm & a PID controller for the arm
  //private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  //private ProfiledPIDController armController = new ProfiledPIDController (0, 0,0,  new TrapezoidProfile.Constraints(0, 0));
  
  //Sets the arm encoder distance per pulse & sets the arm height to 0
  public Arm() {

    //counts every degree of the encoder
  //m_encoder_arm.setDistancePerPulse((1.0/2048.0)*360);
  //seArmHeight(0);

  }
  //public double getArmHeight() {
   // Returns the arm height
    //return m_encoder_arm.getDistance();
  //}
  //Sets the arm to a certain height
  //public void seArmHeight(double degree) {
  //  m_arm.setVoltage(armController.calculate(m_encoder_arm.getDistance()+ armFeedforward.calculate(degree, 0)));
  //}
    
}

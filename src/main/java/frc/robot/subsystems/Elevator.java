// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  //Creates a SparkMax motor controller for the elevator & an encoder for the elevator
private final SparkMax m_elevator = new SparkMax(7, MotorType.kBrushless);
private final RelativeEncoder m_elevatorEncoder = m_elevator.getEncoder();
//private final DigitalInput m_bottomLimitSwitch = new DigitalInput(0);

//Creates a feedforward controller for the elevator & a PID controller for the elevator
  //private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
  //private ProfiledPIDController elevatorController = new ProfiledPIDController (.0, 0,0,  new TrapezoidProfile.Constraints(.1, .5));
  
  //Sets the elevator encoder distance per pulse & sets the elevator height to 0
  public Elevator() {

    m_elevator.configure(
      Configs.Elevator.elevatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  
    //counts every revolution of the encoder
    //m_encoder_elevator.setDistancePerPulse(1.0/2048.0);
  //seElevatorHeight(0);

  //}
  /*public double getElevatorHeight() {
    //Returns the elevator height
    return m_encoder_elevator.getDistance();
  }*/
  //Sets the elevator to a certain height
  /*public void seElevatorHeight(double height) {
    m_elevator.setVoltage(elevatorController.calculate(m_encoder_elevator.getDistance()+ elevatorFeedforward.calculate(height, 0)));
  }*/

  //Homes the elevator and sets the encoder to 0
  /*public void homeElevator() {
    m_elevator.setVoltage(.25);
    new WaitCommand(.5);
    m_elevator.setVoltage(-.1);
    if (m_bottomLimitSwitch.get()) {
      m_encoder_elevator.reset();
      m_elevator.setVoltage(0);
    }*/
  }
  public void setElevatorSpeed(double speed) {
    m_elevator.set(speed);
  }
}

//Copilot is amazing for coding
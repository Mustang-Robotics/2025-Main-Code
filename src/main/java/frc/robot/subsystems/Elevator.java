// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Configs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  //Creates a SparkMax motor controller for the elevator & an encoder for the elevator
private final SparkMax m_elevator = new SparkMax(7, MotorType.kBrushless);
private SparkClosedLoopController elevatorClosedLoopController = m_elevator.getClosedLoopController();
public RelativeEncoder elevatorEncoder = m_elevator.getEncoder();
private double elevatorTarget = 0;
public double SpeedAdjustment = 1;


  public Elevator() {

    m_elevator.configure(
      Configs.Elevator.elevatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    elevatorEncoder.setPosition(0);
  
    //counts every revolution of the encoder
  
  //seElevatorHeight(0);

  }

  public void changeSetpoint(double setpoint) {
    elevatorTarget = setpoint;
  }

  private void moveToSetpoint() {
    
    elevatorClosedLoopController.setReference(
        elevatorTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.35);
  }


  private void ElevatorSpeedAdjustment() {
    SpeedAdjustment = 1 - (elevatorEncoder.getPosition()/5000);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    ElevatorSpeedAdjustment();
  }
}
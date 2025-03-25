// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Climb extends SubsystemBase {
  /** Creates a new Climb_Motor. */

  private final SparkMax m_climb = new SparkMax(8, MotorType.kBrushless);


  public Climb() {
      m_climb.configure(
      Configs.Climb.climbConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public void setClimbSpeed(double speed) {
<<<<<<< HEAD
    m_climb.set(speed);
=======

    if((m_climb.getEncoder().getPosition() < 1.9*100) || 
    (m_climb.getEncoder().getPosition() >= 1.9*100 && speed <= 0)) {
      m_climb.set(speed);
    }else{
      m_climb.set(0);
    }
  }

  public double get_climb_encoder(){
    return m_climb.getEncoder().getPosition();
>>>>>>> 2fe887a (Done?)
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax m_intake = new SparkMax(6, MotorType.kBrushless);
  public Intake() {

    m_intake.configure(
      Configs.Arm.armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

  }

  //Sets the intake speed
  public void setIntakeSpeed(double speed) {
    m_intake.set(speed);
  }
  public double getIntakeCurrent() {
    return m_intake.getOutputCurrent();
  }

 
}

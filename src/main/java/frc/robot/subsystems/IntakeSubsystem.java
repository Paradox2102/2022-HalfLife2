// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonSRX m_intake = new TalonSRX(Constants.k_intake); 

  public IntakeSubsystem() {}

  public void runIntake(double power) {
    m_intake.set(ControlMode.PercentOutput, power); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

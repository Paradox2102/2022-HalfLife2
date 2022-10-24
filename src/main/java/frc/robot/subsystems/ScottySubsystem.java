// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScottySubsystem extends SubsystemBase {
  TalonSRX m_scotty = new TalonSRX(Constants.k_scotty); 

  public ScottySubsystem() {}

  public void runScotty(double power) {
    m_scotty.set(ControlMode.PercentOutput, power); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

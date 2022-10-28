// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_climb = new TalonFX(Constants.k_climber);
  TalonFX m_climbFollower = new TalonFX(Constants.k_climbFollower); 
  
  public ClimberSubsystem() {
    m_climbFollower.follow(m_climb); 

    m_climb.setNeutralMode(NeutralMode.Brake);
    m_climbFollower.setNeutralMode(NeutralMode.Brake);
  }

  public void setClimbPower(double power){
    m_climb.set(ControlMode.PercentOutput, power); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

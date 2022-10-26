// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX m_shooter = new TalonFX(Constants.k_shooter); 
  TalonFX m_shooterFollower = new TalonFX(Constants.k_shooterFollower); 

  // front wheels
  double k_f = 0; 
  double k_p = 0; 
  double k_i = 0; 
  double k_iZone = 0; 

  // back wheels
  double k_Bf = 0; 
  double k_Bp = 0; 
  double k_Bi = 0; 
  double k_BiZone = 0; 

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter.configFactoryDefault(); 
    m_shooterFollower.configFactoryDefault(); 

    m_shooterFollower.follow(m_shooter); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

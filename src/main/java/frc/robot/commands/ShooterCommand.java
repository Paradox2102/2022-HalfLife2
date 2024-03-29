// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  ShooterSubsystem m_shooterSubsystem; 

  double m_frontPower; 
  double m_backPower; 

  public ShooterCommand(ShooterSubsystem shooterSubsystem, double frontPower, double backPower) {
    m_shooterSubsystem = shooterSubsystem; 
    m_frontPower = frontPower; 
    m_backPower = backPower; 

    addRequirements(shooterSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFrontPower(m_frontPower); 
    m_shooterSubsystem.setBackPower(m_backPower); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFrontPower(0); 
    m_shooterSubsystem.setBackPower(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAuto extends CommandBase {
  Drivetrain m_swerve; 
  DriveSubsystem m_driveSubsystem; 
  double k_delay = 1.0;
  Timer m_timer = new Timer();
  // double m_timer = 0;  

  public DriveAuto(DriveSubsystem driveSubsystem) {
    m_swerve = driveSubsystem.getSwerve(); 
    // m_timer = time; 

    addRequirements(driveSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset(); 
    m_timer.start();
    m_swerve.drive(1, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= k_delay;
  }
}

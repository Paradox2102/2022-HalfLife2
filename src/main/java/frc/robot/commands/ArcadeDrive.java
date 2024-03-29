// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private Drivetrain m_swerve;
  private Joystick m_joystick;
  private XboxController m_xbox; 

  private final boolean k_fieldRelative = true; 
  private final static double k_deadZone = 0.02; 

  public ArcadeDrive(Joystick joystick, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystick = joystick;
    m_swerve = driveSubsystem.getSwerve();
    addRequirements(driveSubsystem);
  }

  public ArcadeDrive(XboxController xbox, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xbox = xbox;
    m_swerve = driveSubsystem.getSwerve();
    addRequirements(driveSubsystem);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_joystick.getY(), k_deadZone))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_joystick.getX(), k_deadZone))
    * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_joystick.getZ(), k_deadZone))
    * Drivetrain.kMaxAngularSpeed;

    // final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(0.5, 0.02))
    // * Drivetrain.kMaxAngularSpeed;

    // System.out.println(String.format("xspeed=%f, yspeed=%f, rot=%f, fieldRelative=%b", xSpeed, ySpeed, rot, fieldRelative)); 

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private void driveWithXbox(boolean fieldRelative) {
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftY(), k_deadZone))
    * Drivetrain.kMaxSpeed;

    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftX(), k_deadZone))
    * Drivetrain.kMaxSpeed;

    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_xbox.getRightX(), k_deadZone))
    * Drivetrain.kMaxAngularSpeed;

    // System.out.println(String.format("xspeed=%f, yspeed=%f, rot=%f, fieldRelative=%b", xSpeed, ySpeed, rot, fieldRelative)); 

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_joystick != null){   
      driveWithJoystick(k_fieldRelative);
    }
    else {
      driveWithXbox(k_fieldRelative); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

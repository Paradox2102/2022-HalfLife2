// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.381,
  // 0.381);
  // private final Translation2d m_frontRightLocation = new Translation2d(0.381,
  // -0.381);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.381,
  // 0.381);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.381,
  // -0.381);

  // public static Translation2d m_frontLeftLocation = new Translation2d(0.245,
  // 0.245);
  // public static Translation2d m_frontRightLocation = new Translation2d(0.245,
  // -0.245);
  // public static Translation2d m_backLeftLocation = new Translation2d(-0.245,
  // 0.245);
  // public static Translation2d m_backRightLocation = new Translation2d(-0.245,
  // -0.245);

  public static Translation2d m_frontLeftLocation = new Translation2d(+0.245, -0.245);
  public static Translation2d m_frontRightLocation = new Translation2d(+0.245, +0.245);
  public static Translation2d m_backLeftLocation = new Translation2d(-0.245, -0.245);
  public static Translation2d m_backRightLocation = new Translation2d(-0.245, +0.245);

  public final SwerveModule m_frontLeft;
  public final SwerveModule m_frontRight;
  public final SwerveModule m_backLeft;
  public final SwerveModule m_backRight;

  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  public final Gyro m_gyro = new GyroWrapper(new PigeonIMU(0));

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain(SwerveModule FLMotor, SwerveModule FRMotor, SwerveModule BLMotor, SwerveModule BRMotor) {
    m_frontLeft = FLMotor;
    m_frontRight = FRMotor;
    m_backLeft = BLMotor;
    m_backRight = BRMotor;

    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void resetGyro() {
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  }
}

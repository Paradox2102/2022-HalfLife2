// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    // Use addRequirements() here to declare subsystem dependencies.

    //Swerve Drivetrain IDs
    public final static int k_FLDriveMotor = 1; 
    public final static int k_BLDriveMotor = 7; 
    public final static int k_FRDriveMotor = 3;
    public final static int k_BRDriveMotor = 5; 

    public final static int k_FLTurningMotor = 2; 
    public final static int k_BLTurningMotor = 8; 
    public final static int k_FRTurningMotor = 4; 
    public final static int k_BRTurningMotor = 6; 

    public final static int k_FLDriveEncoder = 0; 
    public final static int k_BLDriveEncoder = 0; 
    public final static int k_FRDriveEncoder = 0; 
    public final static int k_BRDriveEncoder = 0; 

    public final static int k_FLTurningEncoder = 0; 
    public final static int k_BLTurningEncoder = 0; 
    public final static int k_FRTurningEncoder = 0; 
    public final static int k_BRTurningEncoder = 0; 

    public final static double k_RPMtoMPS = (4.616 / 100) / 60;

    //PID Controls 
    public static double k_F = (2500.0 / 2900) / 5000;

    public static double k_P = 0.00002;
    public static double k_I = 0.0000005;
    public static double k_iZone = 150;
    
    //Climb Talon IDs (Used for Absolute Encoders on Swerve Drive)
    public final static int k_climber = 0; 
    public final static int k_climbFollower = 0; 

    //Intake Talon IDs (Used for Absolute Encoders on Swerve Drive)
    public final static int k_intake = 0; 

    //Serializer Talon IDs (Used for Absolute Encoders on Swerve Drive)
    public final static int k_scotty = 0; 

    //Shooter Talon IDs
    public final static int k_shooter = 0; 
    public final static int k_shooterFollower = 0; 

}

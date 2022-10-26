package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetAzimuthZero;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(); 

    Joystick m_stick = new Joystick(0);
//     XboxController m_stick = new XboxController(0); 
    
    JoystickButton m_intake = new JoystickButton(m_stick, 1); 
    JoystickButton m_scotty = new JoystickButton(m_stick, 2); 
    JoystickButton m_shooter = new JoystickButton(m_stick, 3); 
    JoystickButton m_climb = new JoystickButton(m_stick, 3); 

    // The robot's subsystems and commands are defined here...

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_stick, m_driveSubsystem));
        m_driveSubsystem.loadAndSetAzimuthZeroReference();
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putData("SetAzimuthZero", new SetAzimuthZero(m_driveSubsystem));
        SmartDashboard.putData("Reset Gyro", new ResetGyro(m_driveSubsystem));
    }

    private void configureButtonBindings() {
        m_climb.whileHeld(new ClimbCommand(m_climberSubsystem, () -> m_stick.getY())); 
    }

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                3,
                3)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_driveSubsystem.getSwerve().m_kinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                //End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, Rotation2d.fromDegrees(180)),
                config);

        var thetaController = new ProfiledPIDController(
                1, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_driveSubsystem::getPose, // Functional interface to feed supplier
                m_driveSubsystem.getSwerve().m_kinematics,

                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                m_driveSubsystem::setModuleStates,
                m_driveSubsystem);
                

        // Reset odometry to the starting pose of the trajectory.
        m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_driveSubsystem.getSwerve().drive(0, 0, 0, false));
    }

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            Math.PI, Math.PI);
}

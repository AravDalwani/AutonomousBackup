/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.AutonomousCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Joystick joy1 = new Joystick(Constants.JoystickPort);

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem);
  private final AutonomousCommandGroup m_AutonomousCommandGroup = new AutonomousCommandGroup(m_driveSubsystem);

  
  public static Encoder l_enc = new Encoder(2,3,false,EncodingType.k4X);
  public static Encoder r_enc = new Encoder(0,1,true, EncodingType.k4X);

  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
  }

  public static double getY(final Joystick joy, final double band) {
    // Inverted (Joystick moved forwards gives negtive reading)
    double val = -joy.getY();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getZ(Joystick joy, double band) {
    double val = joy.getZ();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutonomousCommandGroup;

  // String trajectoryJSON = "paths/YourPath.wpilib.json";
  // try {
  // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  // Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  // } catch (IOException ex) {
  // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  // }

  //   // An ExampleCommand will run in autonomous
  //   TrajectoryConfig config  = new TrajectoryConfig(Units.feetToMeters(2),Units.feetToMeters(2));
  //   config.setKinematics(m_driveSubsystem.getKinematics());

  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //     // Start at the origin facing the +X direction
  //     new Pose2d(0, 0, new Rotation2d(0)),
  //     // Pass through these two interior waypoints, making an 's' curve path
  //     List.of(new Translation2d(1, 1),new Translation2d(2, -1)),
  //     // End 3 meters straight ahead of where we started, facing forward
  //     new Pose2d(3, 0, new Rotation2d(0)),
  //     // Pass config
  //     config
  // );

  //   RamseteCommand command = new RamseteCommand(
  //     exampleTrajectory,m_driveSubsystem :: getPose,
  //      new RamseteController(2, 0.7),
  //      m_driveSubsystem.getFeedForward(),
  //      m_driveSubsystem.getKinematics(),
  //      m_driveSubsystem::getSpeeds, 
  //      new PIDController(Constants.kP_Autonomous, Constants.kI_Autonomous, Constants.kD_Autonomous),
  //      new PIDController(Constants.kP_Autonomous, Constants.kI_Autonomous, Constants.kD_Autonomous), 
  //      m_driveSubsystem :: setOutput,
  //      m_driveSubsystem
  //      ); 
  //   return command.andThen(() -> m_driveSubsystem.setOutput(0, 0));
  }
}
//
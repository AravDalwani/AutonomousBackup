/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonomousCommands.CommandMoveToAngle;
import frc.robot.commands.AutonomousCommands.CommandMoveToDistance;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 

public class AutonomousCommand extends CommandBase {

  public DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public SolenoidSubsystem m_solenoidSubsystem = new SolenoidSubsystem();
  public CommandMoveToAngle m_commandMoveToAngle;
  /**
   * Creates a new AutonomousCommand.
   */

  double initial;

  public AutonomousCommand(final DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = drive;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.l_enc.reset();
    RobotContainer.r_enc.reset();

    initial = RobotContainer.navx.getYaw();
    
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //og
  public void execute() { 
    // m_driveSubsystem.MoveToDistance(3);
    // try {
    //   Thread.sleep(5000);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    m_driveSubsystem.MoveToAngle(90);

    // new CommandMoveToAngle(m_driveSubsystem);
    // new CommandMoveToDistance(m_driveSubsystem);

    // new SequentialCommandGroup(new CommandMoveToDistance(m_driveSubsystem), new CommandMoveToAngle(m_driveSubsystem));
   
   

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.navx.getYaw()) >= initial + 90;
  }
}

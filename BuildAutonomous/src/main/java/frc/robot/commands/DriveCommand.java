/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */
 

  public DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  public DriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = drive;
    addRequirements(m_driveSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband);
    double zaxis = RobotContainer.getZ(RobotContainer.joy1, Constants.zDeadband);
    m_driveSubsystem.drive.arcadeDrive(yaxis, zaxis);

    SmartDashboard.putNumber("Y - axis ", yaxis);
    SmartDashboard.putNumber("Z - axis ", zaxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

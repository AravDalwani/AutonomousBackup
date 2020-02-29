/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.CommandMoveToAngle;
import frc.robot.commands.AutonomousCommands.CommandMoveToDistance;
import frc.robot.commands.SolenoidCommands.DownCommand;
import frc.robot.commands.SolenoidCommands.UpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousCommandGroup.
   */
  public AutonomousCommandGroup(DriveSubsystem drive, SolenoidSubsystem solenoid) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
    addCommands(
    // new UpCommand(solenoid),
    new CommandMoveToDistance(drive, 3.0)
    // new CommandMoveToAngle(drive, 90),
    // new CommandMoveToDistance(drive, 1.5),
    // new CommandMoveToAngle(drive, 60),
    // new CommandMoveToDistance(drive, 1.0),
    // new DownCommand(solenoid)
    
    );
  }
}

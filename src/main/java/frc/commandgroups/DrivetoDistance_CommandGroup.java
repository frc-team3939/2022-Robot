// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DrivetoDistance_Command;
import frc.robot.commands.ResetDriveEncoderCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetoDistance_CommandGroup extends SequentialCommandGroup {
  /** Creates a new DrivetoDistance_CommandGroup. */
  /**
   * Drives to a specific distance in inches from current position
   * @param distance number of inches that you want to travel (positive values are forward and negitive values are reverse)
   */
  public DrivetoDistance_CommandGroup(double distance) {
    
    double encoder_count = distance * RobotMap.inch_encoder_convert;

    addCommands(new ResetDriveEncoderCommand(), new DrivetoDistance_Command(Robot.drive, encoder_count));
  }
}

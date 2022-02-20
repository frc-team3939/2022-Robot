/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class HomeAngleLimitSwitchCommand extends CommandBase {
  /**
   * Moves shooter angle slowly downwards until it hits limit switch - then sets encoder to 0.
   */
  public HomeAngleLimitSwitchCommand() {
    addRequirements(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.shooter.moveAngle(0.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Robot.shooter.limitCheck();
  }

  // Called once after isFinished returns true
 
  protected void end() {
    Robot.shooter.resetLimit();
    Robot.shooter.moveAngle(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
    Robot.shooter.resetLimit();
    Robot.shooter.moveAngle(0);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class adjustAngleCommand extends CommandBase {
  /**
   * This command changes the angle of the shooter hood
   */
  int a;
  public adjustAngleCommand(int angle) {
    addRequirements(Robot.shooter);
    a = angle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    a = (int) SmartDashboard.getNumber("Shooter Angle", 0); //Change when angle decided upon
    Robot.shooter.angleAdjust(a); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
      return true;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted(){
  }
}

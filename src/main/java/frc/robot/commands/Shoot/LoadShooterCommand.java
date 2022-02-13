/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LoadShooterCommand extends CommandBase {
  double s;
  double d;
  boolean f;
  int i;
  /**
   * 
   * @param speed speed at which to load
   * @param delayInSchedulerCycles delay in scheduler cycles before activating timer (50/s)
   * @param firing whether you want to fire. if false, command stops when activating limit switch - otherwise, powers through it.
   */
  public LoadShooterCommand(double speed, double delayInSchedulerCycles, boolean firing) {
    addRequirements(Robot.shooter);
    s = speed;
    d = delayInSchedulerCycles;
    f = firing;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    i = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    i++;
    if (i > d) {
      Robot.shooter.loaderSpin(s);
    }
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    //return false;
    if (f == false)
      return Robot.shooter.feederLimitCheck();
    else 
      return false;
  }

  // Called once after isFinished returns true
  
  protected void end() {
    Robot.shooter.loaderSpin(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
    end();
  }
}

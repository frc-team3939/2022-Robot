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

public class ShooterSpeedCommand extends CommandBase {
  boolean done;
  double i, fs;
  double ss;
  public ShooterSpeedCommand(double feederspeed, double shooterspeed) {
    addRequirements(Robot.shooter);
    fs = feederspeed;
    ss = shooterspeed;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    i = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    i= i + 1;
    //ss = 70;//(SmartDashboard.getNumber("Speed", 0)); //Delete when speed is decided on
    Robot.shooter.setshooterSpeed(ss); 
    done = SmartDashboard.getBoolean("Shooter Done?", false);

    if( i > 100){
      Robot.shooter.loaderSpin(fs);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (done == true) {
      return true;
    } else {
      return false;
    }

  }

  // Called once after isFinished returns true

  protected void end() {
    Robot.shooter.shooterStop();
    Robot.shooter.loaderSpin(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  protected void interrupted() {
    end();
  }
}

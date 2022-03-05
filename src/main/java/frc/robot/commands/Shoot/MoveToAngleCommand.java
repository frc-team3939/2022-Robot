/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveToAngleCommand extends CommandBase {
  int c;
  boolean d;
  public MoveToAngleCommand(int count, boolean pullFromDashboard) {
    addRequirements(Robot.shooter);
    c = -count;
    d = pullFromDashboard;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if (d == false) {
      Robot.shooter.angleAdjust(-c);
    } else {
      Robot.shooter.angleAdjust(-SmartDashboard.getNumber("Shooter Angle", 0));
    }
     
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (d == true) {
      if ((Robot.shooter.shooterencoder() >=  (-SmartDashboard.getNumber("Shooter Angle", 0)-1)) || (Robot.shooter.shooterencoder() <= (-SmartDashboard.getNumber("Shooter Angle", 0)+1))){
        return true;
      } else {
        return false;
      }
    } else {
      if ((Robot.shooter.shooterencoder() >=  (-c-1)) || (Robot.shooter.shooterencoder() <= (-c+1))){
        return true;
      } else {
        return false;
      }
    }
  }

  // Called once after isFinished returns true
 
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
 
  protected void interrupted() {
    Robot.shooter.moveAngle(0);
  }
}

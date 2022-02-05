/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shoot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AutoShootCommand extends CommandBase {
  double ss;
  int i,ii;
  double p, x;

  public AutoShootCommand() {
    addRequirements(Robot.shooter, Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
    double z = -camtran[2];
    double camerax = 960;
    double cameray = 720;
    double camerafovx = 54;
    double camerafovy = 41;
    double degperpixelx = 0.05625;//camerafovx/camerax;
    double degperpixely = 0.05694;//camerafovy/cameray;
    double targetx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    double targety = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
    double targetdegx = targetx*degperpixelx;
    double targetdegy = targety*degperpixely;
    double radiansx = Math.toRadians(targetdegx);
    double radiansy = Math.toRadians(targetdegy);
    double distancefromx = ((39.25)/Math.tan(radiansx));
    double distancefromy = ((17)/Math.tan(radiansy));
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    SmartDashboard.putNumber("Degrees Per Pixel X", degperpixelx);
    SmartDashboard.putNumber("Degrees Per Pixel Y", degperpixely);
    SmartDashboard.putNumber("Target X Pixels", targetx);
    SmartDashboard.putNumber("Target Y Pixels", targety);
    SmartDashboard.putNumber("Distance Calculated from X", distancefromx);
    SmartDashboard.putNumber("Distance Calculated from Y", distancefromy);
    //SmartDashboard.putNumber("z", z);
    
    ss = 0;

    if (tv == 1.0){

      if (z != 0) {

      if (z < 57.0) {
        //power 60 %
        ss = .6;
        //Robot.shooter.angleAdjust(0);
      } else if (z < 65) {
       //power 60%
       ss = .6;
       //Robot.shooter.angleAdjust(100);
      } else if (z < 79) {
       //power 60%
       ss = .6;
       //Robot.shooter.angleAdjust(200);
      } else if (z < 102) {
        //power 60%
        ss = .6;
        //Robot.shooter.angleAdjust(300);
      } else if (z < 133) {
        //power 60%
        ss = .6;
        //Robot.shooter.angleAdjust(400);
      } else if (z < 181) {
        //power 70%
        ss = .6;
        //Robot.shooter.angleAdjust(500);
      } else if (z < 1000) {
        //power 70%
        ss = .75;
        //Robot.shooter.angleAdjust(450);
      }
      } else {
        ss = -.0087*targetx+2.0268;
        //Robot.shooter.angleAdjust(450);
        /*if (targetx > 142) {
          ss = .75;
          Robot.shooter.angleAdjust(450);
        } else if (targetx > 129) {
          ss = .85;
          Robot.shooter.angleAdjust(450);
          } else if (targetx > 1) {
            ss = .95;
            Robot.shooter.angleAdjust(450);
          }*/
        } 
      }

    Robot.drive.resetEncoder();
    i = 0;
    ii = 0;
    p = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    x = SmartDashboard.getNumber("LimelightX", 0);

    if (Math.abs(x) > 1.0){
      if (ii < 6){
        ii = ii + 1;
      } else {
        ii = 0;
      if (x > 0) {
        p = p + 0.02*Math.abs(x);
    } else {
      p = p - 0.02*Math.abs(x);
    }
  }
  }
    Robot.drive.drivePosition(p, 0);
   
    i= i + 1;
    Robot.shooter.setshooterSpeed(ss); 
    
    if( i > 75){
      Robot.shooter.loaderSpin(1);
    }
    }


  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    Robot.shooter.setshooterSpeed(0);
    Robot.shooter.loaderSpin(0);
  }
}

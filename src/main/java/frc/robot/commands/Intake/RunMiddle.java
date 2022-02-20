// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunMiddle extends CommandBase {
  
  boolean r;
  int d;
  int i;
  boolean dash;
  double s;
  /**
   * 
   * @param reverse reverse the middle motor if true
   * @param delayInSchedulerCycles delay middle motor running, 50 per second
   */
  public RunMiddle(double speed, boolean reverse, int delayInSchedulerCycles, boolean fromDashboard) {
    addRequirements(Robot.intake);
    r = reverse;
    d = delayInSchedulerCycles;
    dash = fromDashboard;
    s = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i++;
    if (dash == false) {
      if (i > d) {
        Robot.intake.runMiddleMotor(s);
      } else {
        Robot.intake.runMiddleMotor(0);
      }
    } else {
      Robot.intake.runMiddleMotor(-SmartDashboard.getNumber("MiddleSpeed", 0));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.runMiddleMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

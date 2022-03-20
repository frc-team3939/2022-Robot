// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class WinchPullPosition extends CommandBase {
  /** Creates a new WinchPullPosition. */
  double p;
  boolean d;
  
  public WinchPullPosition(double position, boolean fromDash) {
    addRequirements(Robot.climber);
    p = position;
    d = fromDash;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (d == false) {
      Robot.climber.goToEncoder(p);
    } else {
      Robot.climber.goToEncoder(SmartDashboard.getNumber("Climber Goto", 0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted == true) 
      Robot.climber.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

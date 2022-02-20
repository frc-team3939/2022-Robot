// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ExtendRetractClimber extends CommandBase {
  /** 
  * @param onoff This boolean controls which way to fire the pnumeatics - true for angle, false for straight
  This function either angles or uprights the moving arms of the climber.
  
  */
  boolean o;
  public ExtendRetractClimber(boolean onoff) {
    addRequirements(Robot.climber);  
    o = onoff;
  }

  // Called when the command is initially scheduled.
  @Override
  // TRUE MEANS FORWARD
  public void initialize() {
    if (o == true) {
      Robot.climber.angleArms();
    } else {
      Robot.climber.uprightArms();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

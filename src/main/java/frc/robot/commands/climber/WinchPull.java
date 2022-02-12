// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class WinchPull extends CommandBase {
  double s;/** Pulls winch at set speed. */
  public WinchPull(double inputSpeed) {
    addRequirements(Robot.climber);
    s = inputSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.climber.pullWinch(s);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.pullWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.climber.checkIfAtLimit() == true) {
      return true;
    } else {
      return false;
    }
  }
}

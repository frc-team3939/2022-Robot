// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ReleaseWinch extends CommandBase {
  /** Releases winch, effectively extending the arm. When interrupted it will lock the motor again such that it can be pulled. */
  public ReleaseWinch() {
    addRequirements(Robot.climber); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.climber.releaseWinch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.lockWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

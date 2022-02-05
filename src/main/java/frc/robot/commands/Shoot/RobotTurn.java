// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RobotTurn extends CommandBase {
  /** Creates a new AutoDriveCommand. */
  // d and x will be inputted from limelight and calculated using the intilizaiton code in autoshoot
  // this whole thing will eventually be merged into it as part of autoshoot
  // for testing purposes currently
  double d;
  double x;
  public RobotTurn(/*int distanceHorizontallyFromTarget, double distanceFromTarget*/) {
    addRequirements(Robot.drive);
    //x = distanceHorizontallyFromTarget;
    //d = distanceFromTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //double distanceToTurnInDegrees = (Math.atan(Math.abs(x)/d)*(180/(Math.PI)));
    //double revolutionsToTurn = (distanceToTurnInDegrees/360);
    /*
    if (d < 0) {
      revolutionsToTurn = -revolutionsToTurn;
    }
    */
    Robot.drive.turnWheels90Right();
    
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
    return false;
  }
}

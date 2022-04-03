// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShootCommandAngle extends CommandBase {
  /** Creates a new Shoot. */
  double ss;
  double a;
  int i;
  public ShootCommandAngle(double shooterspeed, double angle) {
    addRequirements(Robot.shooter);
    addRequirements(Robot.intake);
    ss = shooterspeed;
    a = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    Robot.shooter.setshooterSpeed(ss);
    Robot.shooter.angleAdjust(a);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i= i + 1;
    //ss = 70;//(SmartDashboard.getNumber("Speed", 0)); //Delete when speed is decided on
    Robot.shooter.setshooterSpeed(ss);
    if(i > 70){
      Robot.shooter.feederSpeed(0.5);
      Robot.intake.runMiddleMotor(.55);
    } else if (i > 250) {
      Robot.intake.runMiddleMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.runMiddleMotor(0);
    Robot.shooter.shooterStop();
    Robot.shooter.feederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

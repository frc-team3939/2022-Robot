// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightVisionToggle extends CommandBase {
  /** Creates a new LimelightVisionToggle. */
  boolean t;
  public LimelightVisionToggle(boolean toggle) {
    // Use addRequirements() here to declare subsystem dependencies.
    t = toggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (t == true) {
      if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0) == 0)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      else 
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); 
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
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

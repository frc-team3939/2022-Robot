// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {
  /** Creates a new DriveToDistance. */
  /**
   * Drives the robot to the selected encoder position
   * @param encoder_counts Number of encoder counts to drive (The Value needs to be >= 0)
   */
  public DriveToDistance(double encoder_counts) {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.distancekP, RobotMap.distancekI, RobotMap.distancekD),
        // This should return the measurement
        Robot.drive::getDriveEncoder,
        // This should return the setpoint (can also be a constant)
        encoder_counts,
        // This uses the output
        output -> {
          Robot.drive.drive(-output, 0, 0, 1);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drive);
    getController().setTolerance(0.125);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

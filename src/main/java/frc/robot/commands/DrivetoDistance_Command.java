// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PID_DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetoDistance_Command extends PIDCommand {
  /** Creates a new DrivetoDistance_Command. */
  /**
   * Drives to a specific encoder count
   * @param drive pass in the drivetrain from the robot
   * @param encoder_position encoder position that you want to go to relative to current postion
   */
  public DrivetoDistance_Command(PID_DrivetrainSubsystem drive, double encoder_position) {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.kP, RobotMap.kI, RobotMap.kD),
        // This should return the measurement
        drive::getDriveEncoder,
        // This should return the setpoint (can also be a constant)
        encoder_position,
        // This uses the output
        output -> {
          drive.drive(output, 0, 0, 1);
        });

        addRequirements(Robot.drive);
        getController().setTolerance(2);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

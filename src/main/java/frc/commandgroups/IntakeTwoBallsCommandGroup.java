// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RunMiddleAndIntake;
import frc.robot.commands.Shoot.LoadShooterCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeTwoBallsCommandGroup extends SequentialCommandGroup {
  /** This command group calls the load shooter command (running feeder only) and the entire intake system. However, until the limit switch for the feeder is activated, the intake limit switch is ignored. Once that is activated, the parallel race command group is dropped and the feeder is stopped. Then, the sequential command group will continue to run the intake system until the limit switch is tripped. */
  public IntakeTwoBallsCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new LoadShooterCommand(0.2, 0, false), new RunMiddleAndIntake()), new RunMiddleAndIntake(false));
  }
}

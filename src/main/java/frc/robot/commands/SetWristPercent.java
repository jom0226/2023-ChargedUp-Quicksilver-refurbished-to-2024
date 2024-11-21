// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristPercent extends Command {
  double percent;
  /** Creates a new SetWristPercent. */
  public SetWristPercent(double percent) {
   addRequirements(RobotContainer.wrist);
   this.percent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.wrist.setState(WristState.MANUAL);
    RobotContainer.wrist.setWristPercent(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.wrist.setState(WristState.IDLE);
   RobotContainer.wrist.setWristPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

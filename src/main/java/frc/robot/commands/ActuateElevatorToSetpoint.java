// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ActuateElevatorToSetpoint extends Command {

  double setpoint = 0;
  double tolerance = 0;
  public ActuateElevatorToSetpoint(double setpoint, double tolerance) {
    addRequirements(RobotContainer.elevator);
    this.setpoint = setpoint;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.setSetpoint(setpoint);
    RobotContainer.elevator.setState(ElevatorState.POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.elevator.setState(ElevatorState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevator.isAtSetpoint(false, tolerance);
  }
}

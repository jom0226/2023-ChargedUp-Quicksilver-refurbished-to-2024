// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class ActuateToSetpoint extends Command {
  
double setpointE;
double setpointW;

  /** Creates a new ActuateToSetpoint. */
  public ActuateToSetpoint(double setpointElevator, double setpointWrist) {
    addRequirements(RobotContainer.elevator, RobotContainer.wrist);
    setpointE = setpointElevator;
    setpointW = setpointWrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.setSetpoint(setpointE);
    Wrist.setSetpoint(setpointW);
    RobotContainer.elevator.setState(ElevatorState.POSITION);
    RobotContainer.wrist.setState(WristState.POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.setState(ElevatorState.IDLE);
    RobotContainer.wrist.setState(WristState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
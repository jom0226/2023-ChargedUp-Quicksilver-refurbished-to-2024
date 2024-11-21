// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateElevatorToSetpoint;
import frc.robot.commands.ActuateWristToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActuateLowToHighConeGrid extends SequentialCommandGroup {
  /** Creates a new ActuateLowToHighGrid. */
  public ActuateLowToHighConeGrid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActuateWristToSetpoint(72, 5),
      new ActuateElevatorToSetpoint(87, 30),
      new ActuateWristToSetpoint(15, 5)
    );
  }
}

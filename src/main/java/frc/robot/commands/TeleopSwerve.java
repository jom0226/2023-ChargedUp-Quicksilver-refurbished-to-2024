package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;

public class TeleopSwerve extends Command {

	private double rotation;
	private Translation2d translation;
	private boolean fieldRelative;
	private boolean openLoop;

	private Drive drive;
	private InterpolatedPS4Gamepad controller;

	/**
	 * Driver control
	 */
	public TeleopSwerve(Drive drive, InterpolatedPS4Gamepad controller, boolean fieldRelative, boolean openLoop) {
		this.drive = drive;
		addRequirements(drive);

		this.controller = controller;
		this.fieldRelative = fieldRelative;
		this.openLoop = openLoop;
	}

	@Override
	public void execute() {
		if (RobotState.isTeleop()) {
			double xAxis = -controller.interpolatedLeftYAxis();
			double yAxis = -controller.interpolatedLeftXAxis();
			double rAxis = controller.interpolatedRightXAxis();
			if (RobotContainer.elevator.getPosition() > 42) {
				xAxis *= .5;
				yAxis *= .5;
				rAxis *= .25;
			}
			// (forward/back, left/right) the controller axis is rotated from the
			// Translation 2d axis.
			translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.MAX_SPEED);
			rotation = rAxis * Constants.Swerve.MAX_ANGULAR_VELOCITY;
			drive.drive(translation, rotation, fieldRelative, openLoop);
		}
	}
}
// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.controllers.InterpolatedPS4Gamepad;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.Vision.VisionState;

// public class GridTagTrack extends Command {

// 	private double xAxis = 0;
// 	private double yAxis = 0;
	
// 	private double rotation = 0;
// 	private Translation2d translation;
// 	private boolean fieldRelative;
// 	private boolean openLoop;
// 	private double ySetPoint = 0.0;
// 	private double xSetPoint = -0.66;
// 	private Drive drive;
// 	private Vision vision;
// 	@SuppressWarnings("unused")
// 	private InterpolatedPS4Gamepad controller;

// 	private PIDController xController = new PIDController(0.3, 0.2, 0.0);
// 	private PIDController yController = new PIDController(0.3, 0.2, 0.0);

// 	public GridTagTrack(Drive drive, Vision vision, InterpolatedPS4Gamepad controller, boolean fieldRelative,
// 			boolean openLoop, double ySetPoint) {
// 		this.drive = drive;
// 		this.vision = vision;
// 		addRequirements(drive, vision);

// 		this.controller = controller;
// 		this.fieldRelative = fieldRelative;
// 		this.openLoop = openLoop;
// 		this.ySetPoint = ySetPoint;

// 		// rotController.reset();
// 		// rotController.setIntegratorRange(-0.2, 0.2);
// 		yController.reset();
// 		yController.setIntegratorRange(-0.2, 0.2);
// 		xController.reset();
// 		xController.setIntegratorRange(-0.2, 0.2);
// 	}

// 	@Override
// 	public void initialize() {
// 		vision.setState(VisionState.TRACKING);
// 		vision.getCameraResult();
// 		vision.recieveTarget();
// 		if (vision.result.hasTargets()) {
// 			drive.resetOdometry(new Pose2d(new Translation2d(-vision.getX(), -vision.getY()),
// 					new Rotation2d()));
// 		}
// 	}

// 	@Override
// 	public void execute() {
// 		if (vision.getState().equals(VisionState.TRACKING)) {
// 			yAxis = -1 * yController.calculate(drive.getPose().getY(), ySetPoint);
// 			yAxis = MathUtil.clamp(yAxis, -.4, .4);
// 			System.out.println(yAxis + " Y ");

// 			if (Math.abs(ySetPoint - drive.getPose().getY()) < 0.03) {
// 				yAxis = 0;
// 			}
// 			// yAxis = -controller.interpolatedLeftXAxis();

// 			// xAxis = -1 * xController.calculate(drive.getPose().getX(), xSetPoint);
// 			// xAxis = MathUtil.clamp(xAxis, -.2, .2);
// 			// System.out.println(xAxis + " X ");
// 			// if (Math.abs(xSetPoint - drive.getPose().getX()) < 0.03) {
// 			// 	xAxis = 0;
// 			// }
// 			xAxis = -controller.interpolatedLeftYAxis() * .4;

// 			// (forward/back, left/right) the controller axis is rotated from the Translation 2d axis
// 			translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.MAX_SPEED);
// 			drive.drive(translation, rotation, fieldRelative, openLoop);
// 		}
// 	}

// 	@Override
// 	public boolean isFinished() {
// 		// if (((Math.abs(drive.getPose().getX() - xSetPoint) < .02) && 
// 		if (Math.abs(drive.getPose().getY() - ySetPoint) < .04) {
// 			return true;
// 		}
// 		return false;
// 	}
// }



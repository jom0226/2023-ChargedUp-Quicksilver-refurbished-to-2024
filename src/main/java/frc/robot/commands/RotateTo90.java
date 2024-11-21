package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;

public class RotateTo90 extends Command {

    private double rotation = 0;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private double rotSetPoint = 90;

    private Drive drive;
    private InterpolatedPS4Gamepad controller;

    private PIDController rotController = new PIDController(0.05, 0.5, 0.00);

    /**
     * Driver control
     */
    public RotateTo90(Drive drive, InterpolatedPS4Gamepad controller, boolean fieldRelative, boolean openLoop, double ySetPoint) {
        this.drive = drive;
        addRequirements(drive);
        
        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        rotController.reset();
        rotController.setIntegratorRange(-0.2, 0.2);

        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            rotSetPoint += 180;
        }
      }

    @Override
    public void execute() {
        double xAxis = -controller.interpolatedLeftYAxis();
        double yAxis = -controller.interpolatedLeftXAxis();
        rotation = rotController.calculate(Math.abs(drive.getAngle()), rotSetPoint);
        // rotation += yAxis;
        if (drive.getAngle() < 0) {
            rotation *= -1;
        }
        rotation = MathUtil.clamp(rotation, -5, 5);
        if (Math.abs(rotation) < 0.1) {
            rotation = 0;
        }
        
        translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.MAX_SPEED);
        drive.drive(translation, rotation, fieldRelative, openLoop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}    


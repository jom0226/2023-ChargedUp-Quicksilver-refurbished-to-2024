// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.auto.routines.BlueChargeStationTaxiTwoPieceAuto;
// import frc.robot.auto.routines.BottomThreePieceDispenseAuto;
// import frc.robot.auto.routines.BottomTwoPieceBlueAuto;
// import frc.robot.auto.routines.BottomTwoPieceRedAuto;
// import frc.robot.auto.routines.ChargeStationAuto;
// // import frc.robot.auto.routines.ChargeStationBottomLPath;
// import frc.robot.auto.routines.ChargeStationTaxiAuto;
// // import frc.robot.auto.routines.ChargeStationTaxiGrabAuto;
// // import frc.robot.auto.routines.ChargeStationTopLPath;
// import frc.robot.auto.routines.DoNothing;
// // import frc.robot.auto.routines.RedChargeStationTaxiTwoPieceAuto;
// import frc.robot.auto.routines.SingleHighAuto;
// // import frc.robot.auto.routines.SingleHighTaxiAuto;
// import frc.robot.auto.routines.SingleLowAuto;
// import frc.robot.auto.routines.SingleLowTaxiAuto;
// import frc.robot.auto.routines.TopThreePieceThirdHybridAuto;
// import frc.robot.auto.routines.TopThreePieceThirdKeepAuto;
// import frc.robot.auto.routines.TopThreePieceThirdMidAuto;
// import frc.robot.auto.routines.TopTwoPieceAuto;
// import frc.robot.auto.routines.TopTwoPieceWithBalanceAuto;
// import frc.robot.auto.routines.TopTwoPieceWithGrabAuto;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand;

  @SuppressWarnings("unused")
  private RobotContainer robotContainer;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private SendableChooser<Boolean> resetGyroToggle = new SendableChooser<>();

  

  UsbCamera usbCamera;
  CvSource outputStream;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    SmartDashboard.putData("Auto Selector", autoChooser);
    SmartDashboard.putData("Reset Gyro Toggle", resetGyroToggle);


    // SmartDashboard.putData("Zero Gyro", new ZeroGyroContinuous());

    // autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    // autoChooser.addOption("Charge Station Taxi", new ChargeStationTaxiAuto(RobotContainer.drive));
    // autoChooser.addOption("Charge Station Normal", new ChargeStationAuto(RobotContainer.drive));
    // // autoChooser.addOption("Charge Station Top L Path", new ChargeStationTopLPath(RobotContainer.drive));
    // // autoChooser.addOption("Charge Station Bottom L Path", new ChargeStationBottomLPath(RobotContainer.drive));
    // autoChooser.addOption("Single High", new SingleHighAuto(RobotContainer.drive));
    // // autoChooser.addOption("Single High Taxi", new SingleHighTaxiAuto(RobotContainer.drive));
    // autoChooser.addOption("Single Low", new SingleLowAuto(RobotContainer.drive));
    // autoChooser.addOption("Single Low Taxi", new SingleLowTaxiAuto(RobotContainer.drive));
    // autoChooser.addOption("RED Bottom Two Piece", new BottomTwoPieceRedAuto(RobotContainer.drive));
    // autoChooser.addOption("BLUE Bottom Two Piece", new BottomTwoPieceBlueAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Two Piece", new TopTwoPieceAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Two Piece With Grab", new TopTwoPieceWithGrabAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Three Piece Third Mid", new TopThreePieceThirdMidAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Three Piece Third Hybrid", new TopThreePieceThirdHybridAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Three Piece Third Keep", new TopThreePieceThirdKeepAuto(RobotContainer.drive));
    // autoChooser.addOption("Top Two Piece With Balance", new TopTwoPieceWithBalanceAuto(RobotContainer.drive));
    // autoChooser.addOption("Bottom Three Piece Dispense", new BottomThreePieceDispenseAuto(RobotContainer.drive));
    // autoChooser.addOption("Charge Station Taxi Red Two Piece", new RedChargeStationTaxiTwoPieceAuto(RobotContainer.drive));
    // autoChooser.addOption("Charge Station Taxi Blue Two Piece", new BlueChargeStationTaxiTwoPieceAuto(RobotContainer.drive));
    // autoChooser.addOption("Charge Station Taxi Grab", new ChargeStationTaxiGrabAuto(RobotContainer.drive));


    resetGyroToggle.addOption("YES", true);
    resetGyroToggle.addOption("NO", false);


    usbCamera = CameraServer.startAutomaticCapture();
    outputStream = CameraServer.putVideo("Rectangle", 640, 480);

  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (resetGyroToggle.getSelected() != null) {
    RobotContainer.drive.isResetGyro(resetGyroToggle.getSelected());
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Drive.initialGyroAngle = RobotContainer.drive.getAngle();
    autoCommand = autoChooser.getSelected();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    addPeriodic(
      () -> RobotContainer.drive.swerveOdometry.update(
        RobotContainer.drive.getYaw(),
        RobotContainer.drive.getPositions()),
      0.02);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.claw.setState(ClawState.IDLE);
    RobotContainer.wrist.setState(WristState.IDLE);
    RobotContainer.elevator.setState(ElevatorState.IDLE);
    Wrist.setSetpoint(90);
    Elevator.setSetpoint(0);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

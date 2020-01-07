/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.controllers.XBoxController;
import frc.drive.DriveBase;
import frc.drive.DriveController;
import frc.drive.DriveControllerTrainer;
import frc.shooter.*;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  //upper is normal mode, lower is trainer(dual control) mode
  private DriveController driveController;
  private ShooterControl shooterControl;
  //private DriveControllerTrainer driveController;
  private DriveBase base;
  private Shooter shooter;
  private XBoxController Xbox;
  private XBoxController Xbox2;

  public static boolean debug = false;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Xbox = new XBoxController(0);
    Xbox2 = new XBoxController(1);
    base = new DriveBase();
    shooter = new Shooter();

    //upper is normal mode, lower is trainer(dual control) mode
    driveController = new DriveController(base, Xbox);
    shooterControl = new ShooterControl(shooter, Xbox);
    //driveController = new DriveControllerTrainer(base, Xbox, Xbox2);

    driveController.init();
    shooterControl.init();

    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { //put stuff like motor current/temp pushing here
    
  }

  @Override
  public void autonomousInit() {
    driveController.initAuton();
  }

  @Override
  public void autonomousPeriodic() {
    driveController.updateAuton();
  }

  @Override
  public void teleopPeriodic() {
    driveController.update();
    shooterControl.update();
    shooterControl.printRPM();
  }

  @Override
  public void testPeriodic() {
    if(!debug){debug=true;}
    driveController.update();
  }
}

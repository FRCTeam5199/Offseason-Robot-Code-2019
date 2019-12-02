package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.robot.RobotToggles;
import frc.robot.RobotNumbers;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController{
    private final DriveBase base;
    private final XBoxController controller;

    public DriveController(DriveBase base, XBoxController controller){
        this.base = base;
        this.controller = controller;
    }

    public void init(){
        if(RobotToggles.useDrivePID){base.initPID();}
        else{base.init();}
    }
    
    public void update(){
        //actual drive stuff
        double turnSpeed = controller.getStickRX() * -RobotNumbers.rotationSpeedMultiplier;
        base.drivePID((controller.getStickLY()*(RobotNumbers.forwardSpeedMultiplier)) + turnSpeed, (controller.getStickLY()*(RobotNumbers.forwardSpeedMultiplier)) - turnSpeed);
        if(controller.getRTriggerPressed()){
            base.setBrake(true);
        }
        else{
            base.setBrake(false);
        }
        postData();
    }
    
    //auton/debug information posting
    private void postData(){
        if(RobotToggles.postAnything||Robot.debug){ //can collapse this in vscode so it's not ugly :ok_hand: (oh and it also gives a master toggle)
            if(RobotToggles.postPositionInfo||Robot.debug){
                SmartDashboard.putNumber("L_Feet", base.getFeetLeft());
                SmartDashboard.putNumber("R_Feet", base.getFeetRight());
                SmartDashboard.putNumber("L_FPS", base.getFPSLeft());
                SmartDashboard.putNumber("R_FPS", base.getFPSRight());
            }
            if(RobotToggles.postPositionInfoInches||Robot.debug){
                SmartDashboard.putNumber("L_Inches", base.getInchesLeft());
                SmartDashboard.putNumber("R_Inches", base.getInchesRight());
                SmartDashboard.putNumber("L_IPS", base.getIPSLeft());
                SmartDashboard.putNumber("R_IPS", base.getIPSRight());
            }
            if(RobotToggles.postPigeonPitch||Robot.debug){
                SmartDashboard.putNumber("PitchAbs", base.pitchAbs());
                SmartDashboard.putNumber("PitchRel", base.pitchRel());
            }
            if(RobotToggles.postPigeonYaw||Robot.debug){
                SmartDashboard.putNumber("YawAbs", base.yawAbs());
                SmartDashboard.putNumber("YawRel", base.yawRel());
            }
            if(RobotToggles.postPigeonRoll||Robot.debug){
                SmartDashboard.putNumber("RollAbs", base.rollAbs());
                SmartDashboard.putNumber("RollRel", base.rollRel());
            }
            if(RobotToggles.postMotorDebug||Robot.debug){
                SmartDashboard.putStringArray("Motor Errors", base.checkMotors());
            }
        }
    }
}
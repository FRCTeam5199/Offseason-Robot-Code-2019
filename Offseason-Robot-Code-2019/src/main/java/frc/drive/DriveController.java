package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.robot.RobotToggles;
import frc.robot.RobotNumbers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController{
    private final DriveBase base;
    private final XBoxController controller;

    public DriveController(DriveBase base, XBoxController controller){
        this.base = base;
        this.controller = controller;
    }

    public void init(){
        base.init();
    }
    
    public void update(){
        //actual drive stuff
        double turnSpeed = controller.getStickRX() * -RobotNumbers.rotationSpeedMultiplier;
        base.drive((controller.getStickLY()*(RobotNumbers.forwardSpeedMultiplier)) + turnSpeed, (controller.getStickLY()*(RobotNumbers.forwardSpeedMultiplier)) - turnSpeed);
        if(controller.getRTriggerPressed()){
            base.setBrake(true);
        }
        else{
            base.setBrake(false);
        }
        postData();
    }

    //auton information posting
    private void postData(){
        if(RobotToggles.postAnything){ //can collapse this in vscode so it's not ugly :^)
            if(RobotToggles.postPositionInfo){
                SmartDashboard.putNumber("L_Feet", base.getFeetLeft());
                SmartDashboard.putNumber("R_Feet", base.getFeetRight());
                SmartDashboard.putNumber("L_FPS", base.getFPSLeft());
                SmartDashboard.putNumber("R_FPS", base.getFPSRight());
            }
            if(RobotToggles.postPositionInfoInches){
                SmartDashboard.putNumber("L_Inches", base.getInchesLeft());
                SmartDashboard.putNumber("R_Inches", base.getInchesRight());
                SmartDashboard.putNumber("L_IPS", base.getIPSLeft());
                SmartDashboard.putNumber("R_IPS", base.getIPSRight());
            }
            if(RobotToggles.postPigeonPitch){
                SmartDashboard.putNumber("PitchAbs", base.pitchAbs());
                SmartDashboard.putNumber("PitchRel", base.pitchRel());
            }
            if(RobotToggles.postPigeonYaw){
                SmartDashboard.putNumber("YawAbs", base.yawAbs());
                SmartDashboard.putNumber("YawRel", base.yawRel());
            }
            if(RobotToggles.postPigeonRoll){
                SmartDashboard.putNumber("RollAbs", base.rollAbs());
                SmartDashboard.putNumber("RollRel", base.rollRel());
            }
        }
    }
}
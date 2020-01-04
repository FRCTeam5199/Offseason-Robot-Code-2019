package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.robot.RobotToggles;
import frc.robot.RobotNumbers;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController{
    private final DriveBase base;
    private final XBoxController controller;
    private int autonStage = 0;
    private boolean autonCheck = false;

    private double leftRel;
    private double rightRel;

    private double rightSpeedFactor;
    private double leftSpeedFactor;

    public DriveController(DriveBase base, XBoxController controller){
        this.base = base;
        this.controller = controller;
    }

    public void init(){
        if(RobotToggles.useDrivePID){base.initPID();}
        else{base.init();}
        base.resetEncoders();
        setRelativePositions();
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

    public void initAuton(){
        setRelativePositions();
        autonStage = 0;
        autonCheck = false;
        //base.resetEncoders();
    }

    public void updateAuton(){
        if(!autonComplete()){
            switch(autonStage){
                case 0:
                    if(driveToFeetAuton(3, 3)){
                        setRelativePositions();
                        autonStage++;
                    }
                    break;
                case 1:
                    if(driveToFeetAuton(1,3)){
                        setRelativePositions();
                        autonStage++;
                    }
                    break;
                case 2:
                    if(driveToFeetAuton(3,1)){
                        setRelativePositions();
                        autonStage++;
                    }
                    break;
                default:
                    autonCheck = true;
            }
        }   
        postData();
    }

    public boolean autonComplete(){
        return autonCheck;
    }
    
    private void setRelativePositions(){
        leftRel = base.getFeetLeft();
        rightRel = base.getFeetRight();
    }

    public void prepDriveToFeetAuton(double distLeft, double distRight){
        leftSpeedFactor = 1;
        rightSpeedFactor = 1;
        if(distLeft>distRight){
            rightSpeedFactor = distRight/distLeft;
            leftSpeedFactor = 1;
        }
        if(distRight>distLeft){
            leftSpeedFactor = distLeft/distRight;
            rightSpeedFactor = 1;
        }
    }

    public boolean driveToFeetAuton(double inLeftFeet, double inRightFeet){
        double leftFeet = inRightFeet;
        double rightFeet = inLeftFeet;
        boolean leftComplete = false;
        boolean rightComplete = false;

        if(base.getFeetLeft()<leftRel+leftFeet){
            base.lUpdatePID(RobotNumbers.drivebaseAutoSpeed*5000);
        }
        else{
            base.lUpdatePID(0);
            leftComplete = true;
        }
        if(base.getFeetRight()<rightRel+rightFeet){
            base.rUpdatePID(RobotNumbers.drivebaseAutoSpeed*5000);
        }
        else{
            base.rUpdatePID(0);
            rightComplete = true;
        }
        return rightComplete&&leftComplete;
    }

    //auton/debug information posting
    private void postData(){
        if(RobotToggles.postAnything||Robot.debug){ //can collapse this in vscode so it's not ugly :ok_hand: (oh and it also gives a master toggle)
            if(RobotToggles.postPositionInfo||Robot.debug){
                SmartDashboard.putNumber("L_Feet", base.getFeetLeft());
                SmartDashboard.putNumber("R_Feet", base.getFeetRight());
                SmartDashboard.putNumber("L_FPS", base.getFPSLeft());
                SmartDashboard.putNumber("R_FPS", base.getFPSRight());
                SmartDashboard.putNumber("L_Feet_Rel", leftRel);
                SmartDashboard.putNumber("R_Feet_Rel", rightRel);
                SmartDashboard.putNumber("Auto Stage", autonStage);
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
                //SmartDashboard.putStringArray("Motor Errors", base.checkMotors());
                SmartDashboard.putString("LeaderL", base.checkLL());
                SmartDashboard.putString("LeaderR", base.checkLR());
                SmartDashboard.putString("SlaveL", base.checkS1L());
                SmartDashboard.putString("SlaveR", base.checkS1R());
            }
        }
    }
}
package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.robot.RobotToggles;
import frc.robot.RobotNumbers;
import frc.robot.Robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class DriveController{
    //---------------------------------------------------------------------------------------
    private static final int k_ticks_per_rev = 1024;
    private static final double k_wheel_diameter = 4.0 / 12.0;
    private static final double k_max_velocity = 10;
  
    private static final int k_left_channel = 0;
    private static final int k_right_channel = 1;
  
    private static final int k_left_encoder_port_a = 0;
    private static final int k_left_encoder_port_b = 1;
    private static final int k_right_encoder_port_a = 2;
    private static final int k_right_encoder_port_b = 3;
  
    private static final int k_gyro_port = 0;
  
    private static final String k_path_name = "example";

    private SpeedController m_left_motor;
    private SpeedController m_right_motor;

    private Encoder m_left_encoder;
    private Encoder m_right_encoder;

    private AnalogGyro m_gyro;

    private EncoderFollower m_left_follower;
    private EncoderFollower m_right_follower;

    private Notifier m_follower_notifier;
    //----------------------------------------------------------------------------------
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
        base.resetPigeon();
        //setRelativePositions();
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

    //pathfinder stuff
    public void initAuton(){
        try {
            Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
            Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
        
            m_left_follower = new EncoderFollower(left_trajectory);
            m_right_follower = new EncoderFollower(right_trajectory);
        
            m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
            // You must tune the PID values on the following line!
            m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
            m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
            // You must tune the PID values on the following line!
            m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
            m_follower_notifier = new Notifier(this::followPath);
            m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
        }
        catch (IOException e) {
        e.printStackTrace();
        }
    }

    public void endAuton(){
        m_follower_notifier.stop();
        base.drive(0,0);
    }

    private void followPath() {
        if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
          m_follower_notifier.stop();
        } else {
          double left_speed = m_left_follower.calculate(m_left_encoder.get());
          double right_speed = m_right_follower.calculate(m_right_encoder.get());
          double heading = base.yawRel();
          double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
          double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
          double turn =  0.8 * (-1.0/80.0) * heading_difference;
          base.setLSide(left_speed + turn);
          base.setRSide(right_speed - turn);
        }
    }
    
    /*public void initAuton(){
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
    }*/

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
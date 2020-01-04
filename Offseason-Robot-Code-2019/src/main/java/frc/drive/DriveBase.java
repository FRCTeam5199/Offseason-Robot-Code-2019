package frc.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ControlType;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;
import frc.robot.RobotMap;

public class DriveBase{
    private final PigeonIMU pigeon = new PigeonIMU(0);
    private final CANSparkMax leaderL, leaderR, slaveL, slaveR;

    private boolean usePID = false;

    private static double wheelRadius = 2;
    public double[] ypr = new double[3];
    private double[] startypr = new double[3];

    private CANPIDController lPid;
    private CANPIDController rPid;

    public DriveBase() {
        leaderL = new CANSparkMax(RobotMap.driveLeaderL, MotorType.kBrushless);
        leaderR = new CANSparkMax(RobotMap.driveLeaderR, MotorType.kBrushless);
        slaveL = new CANSparkMax(RobotMap.driveSlaveL, MotorType.kBrushless);
        slaveR = new CANSparkMax(RobotMap.driveSlaveR, MotorType.kBrushless);

        slaveL.follow(leaderL);
        slaveR.follow(leaderR);
    }
    
    //initialization code(setting vars and stuff) ----------------------------------------------------------------------------------------
    public void initPID(){ //do init stuff but with PID startup
        init();
        startPid();
        usePID = true;
    }

    public void resetEncoders(){
        leaderL.getEncoder().setPosition(0);
        leaderR.getEncoder().setPosition(0);
        slaveL.getEncoder().setPosition(0);
        slaveR.getEncoder().setPosition(0);
    }

    public void init(){ //set current limits and idle modes
        leaderL.setSmartCurrentLimit(40);
        leaderR.setSmartCurrentLimit(40);
        slaveL.setSmartCurrentLimit(40);
        slaveR.setSmartCurrentLimit(40);
        leaderL.setIdleMode(IdleMode.kCoast);
        leaderR.setIdleMode(IdleMode.kCoast);
        slaveL.setIdleMode(IdleMode.kCoast);
        slaveR.setIdleMode(IdleMode.kCoast);
    }

    //drive methods ------------------------------------------------------------------------------------------------------------------
    public void setLSide(double speed){ //set speed of left side
        leaderL.set(-speed);
    }
    public void setRSide(double speed){ //set speed of right side
        leaderR.set(speed);
    }

    public void drive(double left, double right){ //drive with normal control or PID depending on if you're initialized with PID
        if(RobotToggles.drive){
            if(!usePID){
                driveNonPID(left, right);
            }
            else if(usePID){
                drivePID(left, right);
            }
        }
    }
    
    public void driveNonPID(double left, double right){ //set speeds of both sides directly
        setLSide(left);
        setRSide(right);
    }

    public void drivePID(double left, double right){ //set target speeds for PID controlled drive
        lUpdatePID(left*5000);
        rUpdatePID(right*5000);
    }

    public void setBrake(boolean brake){ //set idle mode to brake or coast(DOES NOT WORK IN PID DRIVE)
        if(brake == false){
            leaderL.setIdleMode(IdleMode.kCoast);
            leaderR.setIdleMode(IdleMode.kCoast);
            slaveL.setIdleMode(IdleMode.kCoast);
            slaveR.setIdleMode(IdleMode.kCoast);
        }
        else if(brake == true){
            leaderL.setIdleMode(IdleMode.kBrake);
            leaderR.setIdleMode(IdleMode.kBrake);
            slaveL.setIdleMode(IdleMode.kBrake);
            slaveR.setIdleMode(IdleMode.kBrake);
        }
    }

    //pigeon code ------------------------------------------------------------------------------------------------------------------
    public void updatePigeon(){
        pigeon.getYawPitchRoll(ypr);
    }
    public void resetPigeon(){
        updatePigeon();
        startypr = ypr;
    }
    //absolute ypr -----------------------------------------------------------------------------------------------------------------
        public double yawAbs(){ //return absolute yaw of pigeon
            updatePigeon();
            return ypr[0];
        }
        public double pitchAbs(){ //return absolute pitch of pigeon
            updatePigeon();
            return ypr[1];
        }
        public double rollAbs(){ //return absolute roll of pigeon
            updatePigeon();
            return ypr[2];
        }
    //relative ypr ----------------------------------------------------------------------------------------------------------------
        public double yawRel(){ //return relative(to start) yaw of pigeon
            updatePigeon();
            return ypr[0]-startypr[0];
        }
        public double pitchRel(){ //return relative pitch of pigeon
            updatePigeon();
            return ypr[1]-startypr[1];
        }
        public double rollRel(){ //return relative roll of pigeon
            updatePigeon();
            return ypr[2]-startypr[2];
        }

    //drive pid code ------------------------------------------------------------------------------------------------------------------
    private void startPid(){ //create pid controllers for drivebase sides and set the PID constants
        lPid = leaderL.getPIDController();
        rPid = leaderR.getPIDController();
        setPID(RobotNumbers.drivebaseP , RobotNumbers.drivebaseI , RobotNumbers.drivebaseD);
    }
    private void setPID(double p , double i, double d){ //set PID constants
        lPid.setP(p);
        lPid.setI(i);
        lPid.setD(d);

        rPid.setP(p);
        rPid.setI(i);
        rPid.setD(d);

        lPid.setOutputRange(-1, 1);
        rPid.setOutputRange(-1, 1);
    }
    public void lUpdatePID(double rpm){ //set left side PID target speed
        lPid.setReference(-rpm, ControlType.kVelocity);
    }
    public void rUpdatePID(double rpm){ //set right side PID target speed
        rPid.setReference(rpm, ControlType.kVelocity);
    }

    //encoder code ----------------------------------------------------------------------------------------------------------------
    private double wheelCircumference(){
        return 2*wheelRadius*Math.PI;
    }

    //getRotations - get wheel rotations on encoder
    public double getRotationsLeft(){
        return -(leaderL.getEncoder().getPosition())/6.8;
    }
    public double getRotationsRight(){
        return (leaderR.getEncoder().getPosition())/6.8;
    }

    //getRPM - get wheel RPM from encoder
    public double getRPMLeft(){
        return -(leaderL.getEncoder().getVelocity())/6.8;
    }
    public double getRPMRight(){
        return (leaderR.getEncoder().getVelocity())/6.8;
    }

    //getIPS - get wheel IPS from encoder
    public double getIPSLeft(){
        return (getRPMLeft()*wheelCircumference())/60;
    }
    public double getIPSRight(){
        return (getRPMRight()*wheelCircumference())/60;
    }

    //getFPS - get wheel FPS from encoder
    public double getFPSLeft(){
        return getIPSLeft()/12;
    }
    public double getFPSRight(){
        return getIPSRight()/12;
    }

    //getInches - get wheel inches traveled
    public double getInchesLeft(){
        return (getRotationsLeft()*wheelCircumference());
    }
    public double getInchesRight(){
        return (getRotationsRight()*wheelCircumference());
    }

    //getFeet - get wheel feet traveled
    public double getFeetLeft(){
        return (getRotationsLeft()*wheelCircumference()/12);
    }
    public double getFeetRight(){
        return (getRotationsRight()*wheelCircumference()/12);
    }

    //debug code -------------------------------------------------------------------------------------------------------------------
    public String[] checkMotors(){
        String[] output = {checkMotor(leaderL), checkMotor(leaderR), checkMotor(slaveL), checkMotor(slaveR)};
        return output;
    }

    public String checkLL(){
        return checkMotor(leaderL);
    }
    public String checkLR(){
        return checkMotor(leaderR);
    }
    public String checkS1L(){
        return checkMotor(slaveL);
    }
    public String checkS1R(){
        return checkMotor(slaveR);
    }
    public String checkMotor(CANSparkMax motor){
        boolean anyErrors = false;
        String out = motor.toString() + " has error(s): ";
        for(int i=0 ; i<=11 ; i++){
            if(motor.getFault(RobotNumbers.sparkErrorIDs[i])){
                out += RobotNumbers.sparkErrors[i]+", ";
                anyErrors = true;
            }
        }
        if(!anyErrors){
            out += "none";
        }
        else{
            out = out.substring(0, out.length()-3);
        }
        return out;
    }    
}
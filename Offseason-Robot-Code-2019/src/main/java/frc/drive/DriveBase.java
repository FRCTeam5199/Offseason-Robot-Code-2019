package frc.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.RobotMap;

public class DriveBase{
    private final PigeonIMU pigeon = new PigeonIMU(0);
    private final CANSparkMax leaderL, leaderR, slaveL, slaveR;

    public static boolean reverseL = false;
    public static boolean reverseR = false;
    private static double wheelRadius = 2;
    public double[] ypr = new double[3];
    private double[] startypr = new double[3];

    public DriveBase() {
        leaderL = new CANSparkMax(RobotMap.driveLeaderL, MotorType.kBrushless);
        leaderR = new CANSparkMax(RobotMap.driveLeaderR, MotorType.kBrushless);
        slaveL = new CANSparkMax(RobotMap.driveSlaveL, MotorType.kBrushless);
        slaveR = new CANSparkMax(RobotMap.driveSlaveR, MotorType.kBrushless);

        slaveL.follow(leaderL);
        slaveR.follow(leaderR);
    }
    
    //initialization code(setting vars and stuff)
    public void init(){
        leaderL.setSmartCurrentLimit(40);
        leaderR.setSmartCurrentLimit(40);
        slaveL.setSmartCurrentLimit(40);
        slaveR.setSmartCurrentLimit(40);
        leaderL.setIdleMode(IdleMode.kCoast);
        leaderR.setIdleMode(IdleMode.kCoast);
        slaveL.setIdleMode(IdleMode.kCoast);
        slaveR.setIdleMode(IdleMode.kCoast);
    }

    public void setLSide(double speed){
        leaderL.set(-speed);
    }
    public void setRSide(double speed){
        leaderR.set(speed);
    }

    public void drive(double left, double right){
        setLSide(left);
        setRSide(right);
    }

    public void setBrake(boolean brake){
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

    //pigeon code
    public void updatePigeon(){
        pigeon.getYawPitchRoll(ypr);
    }
    public void resetPigeon(){
        updatePigeon();
        startypr = ypr;
    }
    //absolute ypr
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
    //relative ypr
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


    //encoder code
    private double wheelCircumference(){
        return 2*wheelRadius*Math.PI;
    }

    //getRotations - get wheel rotations on encoder
    public double getRotationsLeft(){
        return (leaderL.getEncoder().getPosition())/6.8;
    }
    public double getRotationsRight(){
        return (leaderR.getEncoder().getPosition())/6.8;
    }

    //getRPM - get wheel RPM from encoder
    public double getRPMLeft(){
        return (leaderL.getEncoder().getVelocity())/6.8;
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
}
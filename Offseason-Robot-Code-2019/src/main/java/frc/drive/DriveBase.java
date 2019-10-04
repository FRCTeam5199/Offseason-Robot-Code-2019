package frc.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
public class DriveBase{

    private final CANSparkMax leaderL, leaderR, slaveL, slaveR;
    public boolean reverseL;
    public boolean reverseR;
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
        if(!reverseL){leaderL.set(speed);}
        else if(reverseL){leaderL.set(-speed);}
    }
    public void setRSide(double speed){
        if(!reverseR){leaderR.set(speed);}
        else if(reverseR){leaderR.set(speed);}
    }

    public void drive(double left, double right){
        setLSide(left);
        setRSide(right);
    }
}
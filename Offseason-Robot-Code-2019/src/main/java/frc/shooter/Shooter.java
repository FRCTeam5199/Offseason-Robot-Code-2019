package frc.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ControlType;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.RobotNumbers;
import frc.robot.RobotMap;

public class Shooter{
    private final CANSparkMax shooterL;
    private final CANSparkMax shooterR;

    public Shooter() {
        shooterL = new CANSparkMax(RobotMap.leftShooter, MotorType.kBrushless);
        shooterR = new CANSparkMax(RobotMap.rightShooter, MotorType.kBrushless);

    }

    public void init(){
        shooterL.setSmartCurrentLimit(40);
        shooterR.setSmartCurrentLimit(40);
        shooterL.setIdleMode(IdleMode.kCoast);
        shooterR.setIdleMode(IdleMode.kCoast);
        // shooterL.setInverted(true);
        // shooterL.follow(shooterR);
    }

    public void spin(double speed){
        shooterL.set(-speed);
        shooterR.set(speed);
    }

    public CANEncoder getEncoder(){
        return shooterR.getEncoder();
    }

    public double getRPM(){
        return getEncoder().getVelocity();
    }

    //set RPM with PID later

}
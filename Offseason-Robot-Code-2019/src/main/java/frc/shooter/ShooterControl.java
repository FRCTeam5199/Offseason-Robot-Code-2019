package frc.shooter;

import frc.shooter.*;
import frc.controllers.XBoxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterControl{
    private final Shooter shooter;
    private final XBoxController controller;

    public ShooterControl(Shooter shooter, XBoxController controller){
        this.shooter = shooter;
        this.controller = controller;
    }

    public void init(){
        shooter.init();
    }

    public void update(){
        shooter.spin(controller.getLTrigger());
    }

    public void printRPM(){
        System.out.println(shooter.getRPM());
    }
}
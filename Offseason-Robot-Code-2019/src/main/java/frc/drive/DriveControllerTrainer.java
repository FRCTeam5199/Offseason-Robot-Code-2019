package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.network.Dashboard;

//special DriveController style class that allows control via 2 xbox controllers with one able to be overriden by the other
public class DriveControllerTrainer{
    private final DriveBase base;
    private final XBoxController controller;
    private final XBoxController controllerTeacher;

    private final double rSpeed = -80;
    private final double kSpeed = .8;
    private double speed = 1;

    public DriveControllerTrainer(DriveBase base, XBoxController controller1, XBoxController controller2){
        this.base = base;
        this.controller = controller1;
        this.controllerTeacher = controller2;
    }

    public void init(){
        base.init();
    }
    //controller.getButtonDown(6)
    public void update(){
        double targetSpeed = controller.getStickRX() * rSpeed;
        double turnSpeed = targetSpeed * .01;
        if(!controllerTeacher.getButtonDown(6)){
            base.drive((controller.getStickLY()*(speed)) + turnSpeed, (controller.getStickLY()*(speed)) - turnSpeed);
            if(controller.getRTriggerPressed()){
                base.setBrake(true);
            }
            else{base.setBrake(false);}
        }
        else if(controllerTeacher.getButtonDown(6)){
            base.drive((controllerTeacher.getStickLY()*(speed)) + turnSpeed, (controllerTeacher.getStickLY()*(speed)) - turnSpeed);
            if(controllerTeacher.getRTriggerPressed()){
                base.setBrake(true);
            }
            else{base.setBrake(false);}
        }
    }
}
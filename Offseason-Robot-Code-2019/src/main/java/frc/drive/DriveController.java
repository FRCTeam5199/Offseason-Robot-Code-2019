package frc.drive;
import frc.drive.DriveBase;
import frc.controllers.XBoxController;
import frc.network.Dashboard;

public class DriveController{
    private final DriveBase base;
    private final XBoxController controller;

    private final double rSpeed = 45;
    private final double kSpeed = .8;
    private double speed;

    public DriveController(DriveBase base, XBoxController controller){
        this.base = base;
        this.controller = controller;
    }

    public void init(){
        base.init();
    }
    public void update(){
        double targetSpeed = controller.getStickRX() * rSpeed;
		double turnSpeed = targetSpeed * .01;
        base.drive((controller.getStickLY()*(speed)) + turnSpeed, (controller.getStickLY()*(speed)) - turnSpeed);
    }
}
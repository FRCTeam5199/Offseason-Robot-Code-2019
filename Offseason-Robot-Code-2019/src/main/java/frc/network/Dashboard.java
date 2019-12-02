package frc.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class Dashboard{
    public Dashboard(){
        
    }

    public static List tabs;
    public static List keys;
    public static List datas;

    public static void addDouble(String tab , String key , double data){
        tabs.add(tab);
        keys.add(key);
        datas.add(data);
    }

    public static void updateDashboard(){
        for(int counter = 0 ; counter<keys.size() ; counter++){
            String key = (String)keys.get(counter);
            double number = (double)datas.get(counter);
            SmartDashboard.putNumber(key,number);
        }
    }
}
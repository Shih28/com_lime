package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsysem extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // horizontal degrees
    NetworkTableEntry ty = table.getEntry("ty"); // vertical degrees
    NetworkTableEntry tv = table.getEntry("tv"); // whether there's target or not
    NetworkTableEntry ta = table.getEntry("ta"); // area 0% ~ 100%
    
    // basic variables 
    static double x;
    static double y;
    double area;
    static boolean target;

    // calculate angles
    static double CurrentDegreesofY = Constants.LimelightConstants.LimelightAngle + y;
    static double CurrentRADofY = CurrentDegreesofY*Math.PI/180;
    static double CurrentRADofX = x*Math.PI/180;
    
    // calculate distance to crosshair
    static double DisToCrosshair = Math.pow(Math.pow(Math.atan(CurrentRADofY), 2) + Math.pow(Math.atan(CurrentRADofX), 2), 1/2);


    @Override
    public void periodic(){
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        target = tv.getBoolean(false);
        area = ta.getDouble(0);

        SmartDashboard.putNumber("limelightX", x);
        SmartDashboard.putNumber("limelightY", y);
        SmartDashboard.putNumber("limelightArea", area);
    }

    public static double getDisToCrosshair(){
        return DisToCrosshair;
    }

    public static double getRADX(){
        return CurrentRADofX;
    }

    public static double getRADY(){
        return CurrentRADofY;
    }

    public static boolean getTarget(){
        return target;
    }
}
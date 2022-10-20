package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsysem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LimelightCMD extends CommandBase {

  // subsystems
  private final DriveSubsystem m_drive;
  private final LimelightSubsysem m_limelight;      

  // variables
  private boolean target;
  private double disError;
  private double xError;
  private double disSum;
  private double XSum;
  private double lastX;
  private double lastDis;
  private double disDerivative;
  private double radDerivative;
  private double time = 0.2;

  //PID
  private static double disP = 0.6;
  private static double disI = 0.1;
  private static double disD = 0.1;
  private static double radP = 0.5;
  private static double radI = 0.1;
  private static double radD = 0.1;

 
  public LimelightCMD(DriveSubsystem DriveSub, LimelightSubsysem LimeSub ) {
    m_drive = DriveSub;
    m_limelight = LimeSub;
    addRequirements(DriveSub);
    addRequirements(LimeSub);
  }

  @Override
  public void initialize() {
    disError = LimelightSubsysem.getDisToCrosshair();
    xError = LimelightSubsysem.getRADX();
  }

  @Override
  public void execute() {
    target = LimelightSubsysem.getTarget();
    disError = LimelightSubsysem.getDisToCrosshair();
    xError = LimelightSubsysem.getRADX();

    if(XSum < 100){
      XSum += xError; 
    }

    if(disSum < 1000){
      disSum += disError;
    }

    radDerivative = (xError - lastX)/time;
    disDerivative = (disError - lastDis)/time;

    double turnOutput = xError*radP + XSum*radI + radDerivative*radD;
    double forwardOutput = disError*disP + disSum*disI + disDerivative*disD;

    if(target){
      m_drive.arcadeDrive(forwardOutput, turnOutput);
    }else{
      m_drive.arcadeDrive(0, 0);
    }

    lastDis = disError;
    lastX = xError;
  }


  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      m_drive.arcadeDrive(0, 0);
    }
  }


  @Override
  public boolean isFinished() {
    if(disError < 0.3 || Math.abs(xError) < 0.1){
      return true;
    }
      return false;
  }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // set motors
  private final WPI_TalonSRX LeftFront = new WPI_TalonSRX(Constants.DriveConstants.LeftFrontMotorPort);
  private final WPI_TalonSRX RightFront = new WPI_TalonSRX(Constants.DriveConstants.RightFrontMotorPort);
  private final WPI_TalonSRX LeftBack = new WPI_TalonSRX(Constants.DriveConstants.LeftBackMotorPort);
  private final WPI_TalonSRX RightBack = new WPI_TalonSRX(Constants.DriveConstants.RightBackMotorPort);

  // left motors
  private final MotorControllerGroup LeftMotorGroup = new MotorControllerGroup(LeftFront, LeftBack);
  
  // right motors
  private final MotorControllerGroup RightMotorGroup = new MotorControllerGroup(RightFront, RightBack);

  // the robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(LeftMotorGroup, RightMotorGroup);

  // gyro sensors
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  //odometry class for tracking robot position
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  
  public DriveSubsystem() {
    LeftFront.setInverted(true);
    LeftBack.setInverted(true);
    RightFront.setInverted(true);
    RightBack.setInverted(true);

    resetEncorders();
  }

  //position
  public Pose2d getaPos(){
    return m_odometry.getPoseMeters();
  }

  public void resetEncorders(){
    LeftFront.setSelectedSensorPosition(0);
    RightFront.setSelectedSensorPosition(0);
  }

  public void arcadeDrive(double x, double z){
    m_drive.arcadeDrive(x, z);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

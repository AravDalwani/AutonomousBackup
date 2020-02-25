/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.can.*;


public class DriveSubsystem extends SubsystemBase {
  public static WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.leftMasterPort);
  public static WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.rightMasterPort);
  public static WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.leftSlavePort);
  public static WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.rightSlavePort);


  public double errorSum = 0;
  public double lastError = 0;
  public double lastTimestamp = 0;
  public double iZone = 1;


  SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftMaster, leftSlave);
  SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightMaster, rightSlave);
  
  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.wheelSpacing));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d(5.0, 13.5, new Rotation2d()));

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  Pose2d pose;

  public Rotation2d getHeading()
  {
    return Rotation2d.fromDegrees(-RobotContainer.navx.getAngle());
  }

  public DriveSubsystem() {

    //slave setup
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(false);

    RobotContainer.l_enc.setDistancePerPulse(Constants.kDistancePerPulse);
    RobotContainer.r_enc.setDistancePerPulse(Constants.kDistancePerPulse);
  
    drive.setMaxOutput(Constants.maxOutput);
  } 


  // public static double averageEncoder(){
  //   double encoder = (l_enc.getRate() + r_enc.getRate())/2;
  //   return encoder;
  // }

  // public static double avgDistanceperPulse(){
  //   double avg_DistanceperPulse = (l_enc.getDistancePerPulse() + r_enc.getDistancePerPulse()) / 2;
  //   return avg_DistanceperPulse;
  // }

    public DifferentialDriveWheelSpeeds getSpeeds(){
      return new DifferentialDriveWheelSpeeds(RobotContainer.l_enc.getRate(),RobotContainer.r_enc.getRate());
    }

    public SimpleMotorFeedforward getFeedForward(){
      return feedforward;
    }

    public Pose2d getPose() {
      return pose;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(),RobotContainer.l_enc.getRate(),RobotContainer.r_enc.getRate());
  }

  public void setOutput(double leftVolts, double rightVolts)
  {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

public static double getAverageEncoderPosition() {
  return (RobotContainer.l_enc.getDistance() + RobotContainer.r_enc.getDistance()) / 2;
}

  public void MoveToDistance(double setpoint)
  {
    if(Math.abs(DriveSubsystem.getAverageEncoderPosition()) > setpoint){
      DriveSubsystem.leftMaster.set(0); 
      DriveSubsystem.leftSlave.set(0);
      DriveSubsystem.rightMaster.set(0);
      DriveSubsystem.rightSlave.set(0);
    }
    double sensorPosition = Math.abs(DriveSubsystem.getAverageEncoderPosition());

    // calculations
    double error = setpoint - sensorPosition;
    double timeInterval = Timer.getFPGATimestamp() - lastTimestamp;

    SmartDashboard.putNumber("Error", error);

    if (Math.abs(error) < iZone) {
      errorSum += error * timeInterval;
    }

    double errorRate = (error - lastError) / timeInterval;

    double outputSpeed = Constants.kP_Autonomous * error + Constants.kI_Autonomous* errorSum + Constants.kD_Autonomous * errorRate;

    SmartDashboard.putNumber("PID output", outputSpeed);

    // output to motors
    DriveSubsystem.leftMaster.set(outputSpeed); 
    DriveSubsystem.leftSlave.set(outputSpeed);
    DriveSubsystem.rightMaster.set(-outputSpeed);
    DriveSubsystem.rightSlave.set(-outputSpeed);

    lastError = error;
    lastTimestamp = Timer.getFPGATimestamp();
  }


  public void MoveToAngle(double SetAngle) {
    //RobotContainer.navx.reset();
    double sensorPosition = RobotContainer.navx.getYaw();

    SmartDashboard.putNumber("Heading", RobotContainer.navx.getYaw());

    double error = SetAngle - sensorPosition;
    
    double timeInterval = Timer.getFPGATimestamp() - lastTimestamp;

    SmartDashboard.putNumber("Error", error);

    double outputSpeed =  Math.abs( 0.005 * error );

    SmartDashboard.putNumber("PID output", outputSpeed);

    lastError = error;
    lastTimestamp = Timer.getFPGATimestamp();

    DriveSubsystem.leftMaster.set(outputSpeed); 
    DriveSubsystem.leftSlave.set(outputSpeed);
    DriveSubsystem.rightMaster.set(outputSpeed);
    DriveSubsystem.rightSlave.set(outputSpeed);

    if(RobotContainer.navx.getYaw() > SetAngle)
    {
      DriveSubsystem.leftMaster.set(0); 
      DriveSubsystem.leftSlave.set(0);
      DriveSubsystem.rightMaster.set(0);
      DriveSubsystem.rightSlave.set(0);
    }
  }

public void AutonomousDrive1()
{
  MoveToDistance(10);
}

public DifferentialDriveKinematics getKinematics() {
  return kinematics;
}
}

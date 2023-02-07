// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.testcode.OpenMVJson;
import frc.robot.testcode.OpenMVJson.JsonParseException;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax leftDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftDrivefollow = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightDrive = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightDrivefollow = new CANSparkMax(4, MotorType.kBrushless);
  //private final DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);
  private final PS4Controller controller = new PS4Controller(0);
  private final AHRS gyro = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
  private final RelativeEncoder leftEncoder = leftDrive.getEncoder();
  private final RelativeEncoder rightEncoder = rightDrive.getEncoder();
  private final SparkMaxPIDController leftController = leftDrive.getPIDController();
  private final SparkMaxPIDController rightController = rightDrive.getPIDController();
  private DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.098592, 4.222, 0.24104);
  private final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(0.6895);
  //private final SerialPort OpenMVPort = new SerialPort(115200, Port.kOnboard, 8, Parity.kOdd, StopBits.kOne);
  //private OpenMVJson OpenMVCam = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightDrivefollow.follow(rightDrive);
    leftDrivefollow.follow(leftDrive);
    rightDrive.setInverted(true);
    leftEncoder.setPositionConversionFactor((Math.PI * 0.1524)/16.37);
    rightEncoder.setPositionConversionFactor((Math.PI * 0.1524)/16.37);
    leftEncoder.setVelocityConversionFactor((Math.PI * 0.1524)/(16.37*60));
    rightEncoder.setVelocityConversionFactor((Math.PI * 0.1524)/(16.37*60));
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    leftController.setP(5.9245E-05, 0);
    rightController.setP(5.9245E-05, 0);
    leftController.setD(0, 0);
    rightController.setD(0, 0);
    leftController.setI(0, 0);
    rightController.setI(0, 0);
    leftController.setIZone(0, 0);
    rightController.setIZone(0, 0);
    leftController.setOutputRange(-1, 1, 0);
    rightController.setOutputRange(-1, 1, 0);
    leftController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    rightController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    SmartDashboard.putNumber("P", 5.9245E-05);
    //OpenMVPort.setReadBufferSize(1000);
    //OpenMVPort.setTimeout(0.01);
    //OpenMVPort.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
    //OpenMVPort.setWriteBufferSize(1);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    //SmartDashboard.putNumber("velo", 0);
    odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    Pose2d currentPose = odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    SmartDashboard.putString("gyro", gyro.getRotation2d().toString());
    SmartDashboard.putNumber("left", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right", rightEncoder.getVelocity());
    SmartDashboard.putString("pose", currentPose.toString());
    /*OpenMVCam = null;
    if(OpenMVPort.getBytesReceived() != 0) {
      String content = OpenMVPort.readString();
      if(!content.contains("][")) {
        try {
          OpenMVJson mvinfo = new OpenMVJson(content);
          OpenMVCam = mvinfo;
        }
        catch(JsonProcessingException|JsonParseException e) {
          e.printStackTrace();
        }
      }
      else {
        System.out.println("Double openmvcam reading");
      }
    }
    OpenMVPort.writeString(" ");*/
    /*velo = 0;
    if(controller.getCircleButton() == true) {
      velo = 4/6.0;
    }
    if(controller.getCrossButton() == true) {
      velo = 2/6.0;
    }
    if(controller.getSquareButton() == true) {
      velo = 3/6.0;
    }
    if(controller.getTriangleButton() == true) {
      velo = 1/6.0;
    }*/
    //double leftVelo = controller.getLeftY()*-3;
    //double rightVelo = controller.getRightY()*-3;
    DifferentialDriveWheelSpeeds speeds = differentialDriveKinematics.toWheelSpeeds(new ChassisSpeeds(-controller.getLeftY(), 0, -controller.getRawAxis(4)*(Math.PI/2)));
    double leftVelo = speeds.leftMetersPerSecond;
    double rightVelo = speeds.rightMetersPerSecond;
    leftController.setReference(leftVelo, ControlType.kVelocity, 0, feedforward.calculate(leftVelo));
    rightController.setReference(rightVelo, ControlType.kVelocity, 0, feedforward.calculate(rightVelo));
    SmartDashboard.putNumber("Lvelo", leftVelo);
    SmartDashboard.putNumber("Rvelo", rightVelo);
    SmartDashboard.putNumber("rotational speed", gyro.getRate());
    SmartDashboard.putNumber("target rotational speed", -controller.getRawAxis(4)*(Math.PI/2));
    
    //robotDrive.tankDrive(-0.5*controller.getLeftY(), -0.5*controller.getRightY());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.testcode.OpenMVJson;
import frc.robot.testcode.OpenMVJson.JsonParseException;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax leftDrive = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax rightDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);
  private final PS4Controller controller = new PS4Controller(0);
  private final SerialPort OpenMVPort = new SerialPort(115200, Port.kOnboard, 8, Parity.kOdd, StopBits.kOne);
  private OpenMVJson OpenMVCam = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightDrive.setInverted(true);

    OpenMVPort.setReadBufferSize(1000);
    OpenMVPort.setTimeout(0.01);
    OpenMVPort.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
    OpenMVPort.setWriteBufferSize(1);
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
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    OpenMVCam = null;
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
    OpenMVPort.writeString(" ");
    robotDrive.tankDrive(controller.getLeftY(), controller.getRightY());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

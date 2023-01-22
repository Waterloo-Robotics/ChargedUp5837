// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  VictorSP driveRight1 = new VictorSP(9);
  VictorSP driveRight2 = new VictorSP(8);
  VictorSP driveLeft1 = new VictorSP(7);
  VictorSP driveLeft2 = new VictorSP(6);
  CANSparkMax neo = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder encoder = neo.getEncoder();
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(driveLeft1, driveLeft2);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(driveRight1, driveRight2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_controller = new Joystick(0);
  Joystick bbRight = new Joystick(2);
  Joystick bbLeft = new Joystick(3);
  private final Timer m_timer = new Timer();
  ADIS16470_IMU imu = new ADIS16470_IMU();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    neo.getEncoder().setPosition(0);
    while (neo.getEncoder().getPosition() < 100) {
      // Drive forwards half speed, make sure to turn input squaring off
      // m_robotDrive.arcadeDrive(-1, 0.0, false);
      neo.set(1);
      System.out.println(neo.getEncoder().getPosition());
      System.out.println(neo.getEncoder().getVelocity());
    }

    while (!m_controller.getRawButton(1)) {

      neo.set(0);
      System.out.println(neo.getEncoder().getPosition());

    }

    while (neo.getEncoder().getPosition() > 0) {

      neo.set(-1);
      System.out.println(neo.getEncoder().getPosition());
    }

    while (!m_controller.getRawButton(1)) {

      neo.set(0);
      System.out.println(neo.getEncoder().getPosition());

    }

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    neo.getEncoder().setPosition(0);
    imu.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_controller.getY(), m_controller.getZ());

    // neo.set(m_controller.getY());

    // if (m_controller.getRawButton(1)) neo.getEncoder().setPosition(0);

    System.out.println("Z Axis: " + imu.getAngle());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

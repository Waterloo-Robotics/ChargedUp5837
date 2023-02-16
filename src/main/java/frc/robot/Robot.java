// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

// import java.beans.Encoder;

// import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
// import com.revrobotics.EncoderType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX driveRight1 = new WPI_TalonSRX(6);
  WPI_TalonSRX driveRight2 = new WPI_TalonSRX(7);
  WPI_TalonSRX driveLeft1 = new WPI_TalonSRX(8);
  WPI_TalonSRX driveLeft2 = new WPI_TalonSRX(9);
  CANSparkMax base1 = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax base2 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax secondary = new CANSparkMax(12, MotorType.kBrushless);
  MotorControllerGroup base = new MotorControllerGroup(base1, base2);
  // CANSparkMax neo = new CANSparkMax(4, MotorType.kBrushless);
  // SparkMaxAbsoluteEncoder encoder2 = neo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  // RelativeEncoder encoder = neo.getEncoder();
  
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(driveLeft1, driveLeft2);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(driveRight1, driveRight2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(1);
  Joystick bbRight = new Joystick(0);
  Joystick bbLeft = new Joystick(3);
  private final Timer m_timer = new Timer();
  // ADIS16470_IMU imu = new ADIS16470_IMU();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // imu.calibrate();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive.setInverted(true);
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
    // neo.getEncoder().setPosition(0);
    // while (neo.getEncoder().getPosition() < 100) {
      // Drive forwards half speed, make sure to turn input squaring off
      // m_robotDrive.arcadeDrive(-1, 0.0, false);
      // neo.set(1);
      // System.out.println(neo.getEncoder().getPosition());
      // System.out.println(neo.getEncoder().getVelocity());
    // }

    // while (!m_controller.getRawButton(1)) {

      // neo.set(0);
      // System.out.println(neo.getEncoder().getPosition());

    // }

    // while (neo.getEncoder().getPosition() > 0) {

      // neo.set(-1);
      // System.out.println(neo.getEncoder().getPosition());
    // }

    // while (!m_controller.getRawButton(1)) {

      // neo.set(0);
      // System.out.println(neo.getEncoder().getPosition());

    // }

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    // neo.getEncoder().setPosition(0);
    // imu.setYawAxis(ADIS16470_IMU.IMUAxis.kY);

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_controller.getLeftY(), 1 * m_controller.getRightX());
    // encoder2 = neo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // neo.set(m_controller.getY());

    // if (m_controller.getRawButton(1)) neo.getEncoder().setPosition(0);

    // System.out.println("Encoder: " + encoder2.getPosition());
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    if (bbRight.getRawButton(1)) setArmCoordinates(36, 15);

  }

  double joint1Angle, joint2Angle = 0;
  double c = 0;
  double angleY, angleA = 0;

  double a = 33;
  double b = 42;

  double joint2Calc;

  public void setArmCoordinates(double x, double y) {

    if (x < 0) {

      base.setInverted(true);
      secondary.setInverted(true);
      
      x = Math.abs(x);

    } else {

      base.setInverted(false);
      secondary.setInverted(false);

    }

    c = Math.sqrt((x * x) + (y * y));
    angleY = Math.asin(y/c);
    double ACalc = ((b * b) + (c * c) - (a * a)) / (2 * c * b); // calculate cosine angle A
//        System.out.println("ACalc (before limit)", ACalc);
//        if (ACalc < -1) ACalc = -1; else if (ACalc > 1) ACalc = 1;
//        if (ACalc > 1) {

//            A = 0 - Math.acos(ACalc - 1.0);
//            A = 0;

//        } else {

//            angleA = Math.acos(ACalc);

//        }
//        System.out.println("ACalc", ACalc);
    angleA = Math.acos(ACalc); // Find angle A
    joint1Angle = (Math.PI / 2) - (angleY + angleA);

    double cosineJoint2 = ((a * a) + (b * b) - (c * c)) / (2 * a * b); // calculate cosine Joint2 or angle C
    joint2Angle = Math.acos(cosineJoint2);

    System.out.println("Joint 1 Angle: " + Math.toDegrees(joint1Angle));
//        System.out.println("Sin(A - a little bit)", Math.sin(A - 0.00002));

//        if (Math.sin(A) < Math.sin(A - 0.00002)) {

//        joint2Angle = (Math.PI / 2.0) - Math.asin(joint2Calc) + (Math.PI / 2.0);

//        } else {

//            joint2Angle = Math.asin(joint2Calc);

//        }

    System.out.println("Joint 2 Angle: " + Math.toDegrees(joint2Angle));
//
//        if (Math.asin(joint2Calc) < Math.asin(joint2Calc - 0.0002)) {
//
//
//
//        } else {
//
//            joint2Angle = Math.asin(joint2Calc);
//
//        }

//        System.out.println("Y", Y);
//        System.out.println("A", A);
//        System.out.println("Y + A", Y + A);
//        System.out.println("effectiveLength", c);


  }

}

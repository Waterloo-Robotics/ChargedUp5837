// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

// import java.beans.Encoder;

// import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
// import com.revrobotics.EncoderType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /* Drive Base Motor Controllers */
  WPI_TalonSRX driveRight1 = new WPI_TalonSRX(6);
  WPI_TalonSRX driveRight2 = new WPI_TalonSRX(7);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(driveRight1, driveRight2);

  WPI_TalonSRX driveLeft1 = new WPI_TalonSRX(8);
  WPI_TalonSRX driveLeft2 = new WPI_TalonSRX(9);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(driveLeft1, driveLeft2);

  /* Arm Motor Controllers */
  CANSparkMax m_Joint1_1 = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax m_Joint1_2 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax m_Joint2 = new CANSparkMax(12, MotorType.kBrushless);
  MotorControllerGroup m_Joint1 = new MotorControllerGroup(m_Joint1_1, m_Joint1_2);

  Encoder joint1Enc = new Encoder(0, 1);
  Encoder joint2Enc = new Encoder(2, 3);

  double joint1kP = 0;
  double joint1kI = 0;
  double joint1kD = 0;
  PIDController joint1PID = new PIDController(joint1kP, joint1kI, joint1kD);

  double joint2kP = 0;
  double joint2kI = 0;
  double joint2kD = 0;
  PIDController joint2PID = new PIDController(joint2kP, joint2kI, joint2kD);
  // CANSparkMax neo = new CANSparkMax(4, MotorType.kBrushless);
  // SparkMaxAbsoluteEncoder encoder2 = neo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  // RelativeEncoder encoder = neo.getEncoder();
  
  
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(1);
  Joystick bbRight = new Joystick(0);
  Joystick bbLeft = new Joystick(3);
  private final Timer m_timer = new Timer();
  public enum ArmState {

    ballPickup, cubePickup,
    ballScoreGround, cubeScoreGround,
    ballScoreLow, cubeScoreLow,
    ballScoreHigh, cubeScoreHigh,
    home, homeSequence

  }

  public ArmState armState = ArmState.home;
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
    // Uncomment for driving
//    m_robotDrive.arcadeDrive(m_controller.getLeftY(), 1 * m_controller.getRightX());

    if (m_controller.getAButton()) armState = ArmState.ballPickup;

    this.updateArm();


    // nick arm stuff
     /*double y1Math, y2Math;

     double y1Axis = m_controller.getLeftY();
     double y2Axis = m_controller.getRightY();


     if (Math.abs(y1Axis) < 0.25){
       y1Math = 0.0;
     }
     else {
         y1Math = Math.signum(y1Axis)*(Math.abs(y1Axis) - .25)*1.333;
     }

     if (Math.abs(y2Axis) < 0.25){
       y2Math = 0.0;
     }
     else {
       y2Math = Math.signum(y2Axis)*(Math.abs(y2Axis) - .25)*1.333;
     }

     y1Math = y1Math * 0.25;
     y2Math = y2Math * 0.25;

    Arm Testing
     m_Joint1.set(y2Math);
     m_Joint2.set(y1Math);

     SmartDashboard.putNumber("Arm I", y2Math);
     SmartDashboard.putNumber("Arm D", y1Math);*/



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

  double x, y = 0;
  double joint1Pos, joint2Pos = 0;
  double joint1Speed, joint2Speed = 0;
  public void updateArm() {

    switch (armState) {

      case ballPickup:
        x = 30;
        y = 5;
//         joint1PID.setSetpoint(joint1Angle);
        break;

      case home:
        x = 0;
        y = 0;
        break;

    }
    setArmCoordinates(x, y);
//    joint1PID.setSetpoint(joint1Angle);
//    joint2PID.setSetpoint(joint2Angle);

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("joint1Angle", joint1Angle);
    SmartDashboard.putNumber("joint2Angle", joint2Angle);

//    joint1Pos = joint1Enc.get() * 2 * Math.PI;
//    joint2Pos = joint2Enc.get() * 2 * Math.PI;

//    joint1Speed = joint1PID.calculate(joint1Pos);
//    joint2Speed = joint2PID.calculate(joint2Pos);

//     m_Joint1.set(joint1Speed);
//     m_Joint2.set(joint2Speed);

  }

  double joint1Angle, joint2Angle = 0;
  double c = 0;
  double angleY, angleA = 0;

  double a = 33;
  double b = 42;

  double joint2Calc;

  public void setArmCoordinates(double x, double y) {

    if (x < 0) {

      m_Joint1.setInverted(true);
      m_Joint2.setInverted(true);
      
      x = Math.abs(x);

    } else {

      m_Joint1.setInverted(false);
      m_Joint2.setInverted(false);

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

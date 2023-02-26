// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.Arm.ArmControlState;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.IntakeState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /* Drive Base Motor Controllers */
  public static WPI_TalonSRX m_driveRight1 = new WPI_TalonSRX(6);
  WPI_TalonSRX m_driveRight2 = new WPI_TalonSRX(7);
  private final MotorControllerGroup mg_rightDrive = new MotorControllerGroup(m_driveRight1, m_driveRight2);

  WPI_TalonSRX m_driveLeft1 = new WPI_TalonSRX(8);
  WPI_TalonSRX m_driveLeft2 = new WPI_TalonSRX(9);
  private final MotorControllerGroup mg_leftDrive = new MotorControllerGroup(m_driveLeft1, m_driveLeft2);
  
  // private final DifferentialDrive r_robotDrive = new DifferentialDrive(mg_leftDrive, mg_rightDrive);
  private final XboxController c_controller = new XboxController(1);
  Joystick bbRight = new Joystick(2);
  Joystick bbLeft = new Joystick(3);
  private final Timer timer = new Timer();

  public static SensorCollection Joint2Enc = Robot.m_driveRight1.getSensorCollection();

  Arm arm = new Arm(m_driveRight1);

  public Arm.ArmState armState = Arm.ArmState.home;
  public Arm.IntakeState intakeState = IntakeState.cubeOpen;
  // ADIS16470_IMU imu = new ADIS16470_IMU();

  ArmPosition home = new ArmPosition(0, 9, 0);
  ArmPosition homeFront = new ArmPosition(16, 9, 0);
  ArmPosition homeBack = new ArmPosition(-16, 9, 0);
  ArmPosition conePickupFront = new ArmPosition(30, 5, 0);
  ArmPosition conePickupBack = new ArmPosition(-30, 5, 0);
  ArmPosition coneScoreFront = new ArmPosition(38, 10, 0);
  ArmPosition coneScoreBack = new ArmPosition(-38, 10, 0);

  boolean isAuto = false;
  boolean movingAuto = false;

  ArmPosition currentArmPosition = new ArmPosition(0, 9, 0);
  ArmPosition desiredArmPosition = new ArmPosition(0, 9, 0);
  boolean newDesiredArmPosition;

  ArmPathPlanner pathPlanner = new ArmPathPlanner(homeFront, homeBack);
  ArmSequence currentSequence = new ArmSequence();
  
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
    mg_leftDrive.setInverted(true);
    Arm.mg_Joint1.setInverted(true);
    // Arm.m_Joint1_1.getEncoder().setInverted(true);
  }

  public void robotPeriodic() {

    SmartDashboard.putNumber("Joint 3 Enc", arm.joint3CurrentPosition());
    SmartDashboard.putNumber("Sequence Steps", currentSequence.getLength());
    SmartDashboard.putNumber("Current Step", currentSequence.currentIndex);
    SmartDashboard.putNumber("Current Side", currentArmPosition.returnSide());
    SmartDashboard.putNumber("Desired Side", desiredArmPosition.returnSide());

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    Arm.isHomed = false;
    isAuto = false;
    currentArmPosition.setCoordinates(0, 9, 0);

  }
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    /** CONTROLS:
     * Driver 1 Closes and opens claw, Driver one controls what side of the claw is used for pickup
     * Driver 1 (and 2?) controls joint 3
     *
     * Driver 2 controls arm automatic positioning and manual adjustments (all through inverse kinematics)
     * Driver 2 controls initial joint 3 position
     * */

    // get drive joysticks

    // Dead Zone calculations

    // Update Drivebase

    // Claw Control
      // rotate claw
      // Actuate pistons

    // Arm Position controls
      // TODO if robot drive fast for extended period of time (a few seconds?) set arm to position within bumpers

    // Uncomment for driving
//    m_robotDrive.arcadeDrive(m_controller.getLeftY(), 1 * m_controller.getRightX());

if (bbLeft.getRawButton(1)) intakeState = IntakeState.cubeOpen;
if (bbLeft.getRawButton(4)) intakeState = IntakeState.cubeClosed;
if (bbRight.getRawButton(1)) intakeState = IntakeState.coneIntakeBack;
if (bbRight.getRawButton(4)) intakeState = IntakeState.coneIntakeForward;
if (bbRight.getRawButton(2)) intakeState = IntakeState.coneClosed;
arm.updateIntake(intakeState);

  if (bbLeft.getRawButton(10)) {armState = ArmState.goHome; movingAuto = true;}
  if (bbRight.getRawButton(8)) {armState = ArmState.goConePickup; movingAuto = true;}
  if (c_controller.getBButtonPressed()) {armState = ArmState.goConeScoreLow; movingAuto = true;}
  if (bbRight.getRawButton(10)) {armState = ArmState.goFrontHome; movingAuto = true;}
  if (bbRight.getRawButton(9)) {armState = ArmState.goBackHome; movingAuto = true;}
  if (bbRight.getRawButton(5)) {armState = ArmState.goConeScoreGround; movingAuto = true;}
  if (bbRight.getRawButton(6)) {armState = ArmState.goConeScoreLow; movingAuto = true;}

  if (c_controller.getBackButtonPressed() && Arm.isHomed) {

    currentArmPosition.setCoordinates(0, 9, 0);
    isAuto = !isAuto;

  }

  switch (armState) {

    case goConePickup:
      desiredArmPosition = conePickupFront;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.conePickup;
      break;

    case goHome:
      desiredArmPosition = home;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.home;
      break;
    
    case goFrontHome:
      desiredArmPosition = homeFront;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.frontHome;
      break;

      case goConeScoreGround:
      desiredArmPosition = coneScoreFront;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreGround;
      break;

    case goConeScoreLow:
      desiredArmPosition = coneScoreBack;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreLow;
      break;

    case goBackHome:
      desiredArmPosition = homeBack;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.backHome;
      break;

    default:
    break;

  }

  if (isAuto) {

    if (bbLeft.getRawButton(2)) currentArmPosition.incrementZ(-2);
    else if (bbLeft.getRawButton(3)) currentArmPosition.incrementZ(2);
    if (currentArmPosition.z > 225) currentArmPosition.z = 225;
    else if (currentArmPosition.z < -225) currentArmPosition.z = -225;
    if (bbRight.getRawButton(5)) currentArmPosition.z = 0;

    if (Math.abs(bbRight.getY()) > 0.5 || 
        Math.abs(bbRight.getX()) > 0.5 || 
        bbLeft.getRawButton(2) || 
        bbLeft.getRawButton(3)) {
      
      armState = ArmState.manual;
      Arm.armControlState = ArmControlState.manual;
    
    }

    currentArmPosition.incrementX(deadZone(bbRight.getX()) * 0.25);
    currentArmPosition.incrementY(deadZone(bbRight.getY()) * -0.25);

    if (currentArmPosition.x < 16 && currentArmPosition.x > -16) {
      
      currentArmPosition.y = 9;
      currentArmPosition.z = 0;
    
    }
    
    SmartDashboard.putBoolean("Is Valid", arm.isArmPositionValid(currentArmPosition.x, currentArmPosition.y));
    SmartDashboard.putNumber("Attempted x", currentArmPosition.x);
    SmartDashboard.putNumber("Attempted y", currentArmPosition.y);

    if (!arm.isArmPositionValid(currentArmPosition.x, currentArmPosition.y) && armState == ArmState.manual) {

      currentArmPosition.x = Arm.lastValidX;
      currentArmPosition.y = Arm.lastValidY;

    } else if (armState == ArmState.manual) {

      Arm.lastValidX = currentArmPosition.x;
      Arm.lastValidY = currentArmPosition.y;

    }

    SmartDashboard.putNumber("Valid x", currentArmPosition.x);
    SmartDashboard.putNumber("Valid y", currentArmPosition.y);

    if (bbRight.getRawButtonPressed(1)) Arm.m_Joint3.getEncoder().setPosition(0);

    if (Arm.armControlState == ArmControlState.manual) {
    
    } else if (Arm.armControlState == ArmControlState.newPath) {

      Arm.armControlState = ArmControlState.runningPath;
      currentSequence = pathPlanner.planPath(currentArmPosition, desiredArmPosition);
      currentArmPosition = currentSequence.nextPosition();

    } else if (Arm.armControlState == ArmControlState.runningPath) {

      if (arm.isArmInPosition() && !currentSequence.sequenceFinished()) {

        currentArmPosition = currentSequence.nextPosition();

      } else if (arm.isArmInPosition() && currentSequence.sequenceFinished()) {

        Arm.armControlState = ArmControlState.completedPath;

      }

    }

    arm.updateArm(currentArmPosition);

  } else {

    Arm.joint1Brake.set(Value.kForward);
    Arm.joint2Brake.set(Value.kForward);
    
    arm.setJoint1(deadZone(c_controller.getLeftX()) * 0.1);
    arm.homeJoint1(c_controller.getAButtonPressed());
    arm.setJoint2(deadZone(c_controller.getRightX()) * 0.075);

  }

  SmartDashboard.putBoolean("Is joint 1 at position", arm.isJoint1AtPosition());
  SmartDashboard.putBoolean("Is joint 2 at position", arm.isJoint2AtPosition());
  SmartDashboard.putBoolean("Auto Status", isAuto);
  SmartDashboard.putBoolean("Homed", arm.isHomed);
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {



  }

  public double deadZone(double input) {

    if (input < 0.1 && input > -0.1) return 0;
    else return input;

  }

}

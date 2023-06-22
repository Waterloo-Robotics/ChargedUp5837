// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.ObjectInputStream.GetField;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ArmControlState;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.IntakeState;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

    /* Drive Base Motor Controllers */
    public static WPI_TalonSRX m_driveRight1 = new WPI_TalonSRX(6);
    public static WPI_TalonSRX m_driveRight2 = new WPI_TalonSRX(7);
    public static MotorControllerGroup mg_rightDrive = new MotorControllerGroup(m_driveRight1, m_driveRight2);

    public static WPI_TalonSRX m_driveLeft1 = new WPI_TalonSRX(8);
    public static WPI_TalonSRX m_driveLeft2 = new WPI_TalonSRX(9);
    public static MotorControllerGroup mg_leftDrive = new MotorControllerGroup(m_driveLeft1, m_driveLeft2);

    static DifferentialDrive r_robotDrive = new DifferentialDrive(mg_leftDrive, mg_rightDrive);

    /* Driver Input */
    final static XboxController c_controller = new XboxController(1);
    Joystick farmSim1 = new Joystick(4);
    Joystick farmSim2 = new Joystick(5);
    private final Timer timer = new Timer();

    public static SensorCollection Joint1Enc = m_driveLeft2.getSensorCollection();
    public static SensorCollection Joint2Enc = m_driveRight2.getSensorCollection();

    public static SensorCollection leftDriveEnc = m_driveLeft1.getSensorCollection();
    public static SensorCollection rightDriveEnc = m_driveRight1.getSensorCollection();

    Odometry odometry = new Odometry(mg_leftDrive, mg_rightDrive, r_robotDrive, leftDriveEnc, rightDriveEnc);

    Arm arm = new Arm(m_driveLeft2, m_driveRight2);
    int joint1Timeout;
    int joint1TimeoutLimit = 75;
    boolean joint1TimeoutEnable = true;
    int joint2Timeout;
    int joint2TimeoutLimit = 75;
    boolean joint2TimeoutEnable = true;

    boolean brake = false;

    boolean timeoutOverride = false;

    public Arm.ArmState armState = Arm.ArmState.home;
    public Arm.IntakeState intakeState = IntakeState.cubeOpen;
    ADIS16470_IMU imu = new ADIS16470_IMU();

    boolean coneControl, front = true;

    ArmPosition home = new ArmPosition(0, 9, 90);
    ArmPosition homeFront = new ArmPosition(16, 9, 90);
    ArmPosition homeBack = new ArmPosition(-16, 9, 90);

    /* Pickup Positions */
    ArmPosition conePickupFrontGround = new ArmPosition(22, 2.75, 225);
    ArmPosition conePickupBackGround = new ArmPosition(-24.5, -3, -1);

    ArmPosition conePickupFrontShelf = new ArmPosition(33.5, 37.75, 264);
    ArmPosition conePickupBackShelf = new ArmPosition(-36.25, 36.75, -81);

    ArmPosition conePickupOnSideFront = new ArmPosition(25.5, 3.75, -117);
    ArmPosition conePickupOnSideBack = new ArmPosition(-25.5, 3.75, -117);

    ArmPosition cubePickupFrontGround = new ArmPosition(22, -0.25, 195);
    ArmPosition cubePickupBackGround = new ArmPosition(-24.5, -3, -1);

    ArmPosition cubePickupFrontShelf = new ArmPosition(33.5, 37.75, 264);
    ArmPosition cubePickupBackShelf = new ArmPosition(-36.25, 36.75, -81);

    /* Cone Scoring Positions */
    ArmPosition coneScoreFrontLow = new ArmPosition(31.25, 5.5, 183);
    ArmPosition coneScoreFrontMiddle = new ArmPosition(46, 35.25, 215);
    ArmPosition coneScoreFrontHigh = new ArmPosition(50, 50, 252);

    ArmPosition coneScoreBackLow = new ArmPosition(31.25, 5.5, 183);
    ArmPosition coneScoreBackMiddle = new ArmPosition(-46.25, 28.25, -21);
    ArmPosition coneScoreBackHigh = new ArmPosition(-53.75, 43.25, -45);

    /* Cube Scoring Positions */
    ArmPosition cubeScoreFrontLow = new ArmPosition(28.5, 5.25, 213);
    ArmPosition cubeScoreFrontMiddle = new ArmPosition(42.25, 38, -74);
    ArmPosition cubeScoreFrontHigh = new ArmPosition(51.75, 49, -44);

    ArmPosition cubeScoreBackLow = new ArmPosition(-27.75, 1.25, -40);
    ArmPosition cubeScoreBackMiddle = new ArmPosition(-38.25, 31.75, 258);
    ArmPosition cubeScoreBackHigh = new ArmPosition(-49, 42, 249);

    boolean isAuto = false;
    boolean movingAuto = false;

    /* ArmPositions to store where the arm is, and where we want it to go */
    ArmPosition currentArmPosition = new ArmPosition(0, 9, 0);
    ArmPosition desiredArmPosition = new ArmPosition(0, 9, 0);

    /* Global Arm Path Planner */
    ArmPathPlanner pathPlanner = new ArmPathPlanner(homeFront, homeBack);
    /* Global Arm Sequence */
    ArmSequence currentSequence = new ArmSequence();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry camMode;

    AnalogInput distanceSensor = new AnalogInput(0);

    /* Autonomous Variables */
    double score1TimeoutAuto = 0.5;
    int autoStep = 1;

    int autoIntakeCounter = 0;
    int autoArmCounter = 0;
    int autoOdoCounter = 0;

    Timer autoTimeoutTimer = new Timer();

    private static final String defaultAuto = "Default Auto";
    private static final String rightAuto = "Right Auto";
    private static final String leftAuto = "Left Auto";
    private static final String middleAuto = "Middle Auto";
    private static final String middleChargeAuto = "Middle Charge Auto";
    private static final String testAuto = "Test Auto";
    private String autoSelected;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private static final String cubeAuto = "Cube Auto";
    private static final String coneAuto = "Cone Auto";
    private String autoGamePiece;
    private final SendableChooser<String> gamePieceChooser = new SendableChooser<>();


    public static boolean lock = false;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        Arm.m_Joint3.getEncoder().setPosition(24);
        imu.calibrate();
        imu.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        mg_leftDrive.setInverted(true);
        Arm.mg_Joint1.setInverted(true);

        m_driveLeft1.configOpenloopRamp(0.25);
        m_driveLeft2.configOpenloopRamp(0.25);
        m_driveRight1.configOpenloopRamp(0.25);
        m_driveRight2.configOpenloopRamp(0.25);

        // Arm.m_Joint1_1.getEncoder().setInverted(true);

        camMode = table.getEntry("camMode");

        camMode.setNumber(1);
        autoTimeoutTimer.reset();
        autoTimeoutTimer.start();

        autoChooser.setDefaultOption("Default Auto", defaultAuto);
        autoChooser.addOption("Right Auto", rightAuto);
        autoChooser.addOption("Left Auto", leftAuto);
        autoChooser.addOption("Middle Auto", middleAuto);
        autoChooser.addOption("Middle Charge Auto", middleChargeAuto);
        autoChooser.addOption("Test Auto", testAuto);
        SmartDashboard.putData("Auto choices", autoChooser);

        gamePieceChooser.setDefaultOption("Cube Auto", cubeAuto);
        gamePieceChooser.addOption("Cone Auto", coneAuto);
        SmartDashboard.putData("Game Piece", gamePieceChooser);

    }

    public void robotPeriodic() {
        /************ Arm Smart Dashboard ************/
        /* Current Angles */
        SmartDashboard.putNumber("Distance Sensor Value", distanceSensor.getValue());
        SmartDashboard.putNumber("Joint 1 Angle", arm.joint1CurrentPosition());
        SmartDashboard.putNumber("Joint 2 Angle", arm.joint2CurrentPosition());
        SmartDashboard.putNumber("Joint 3 Enc", arm.joint3CurrentPosition());
        SmartDashboard.putNumber("Joint 3 Raw", Arm.m_Joint3.getEncoder().getPosition());

        /* Desired Angles */
        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(Arm.joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(arm.joint2SetPoint));
        SmartDashboard.putNumber("Joint 3 Desired Angle", Arm.joint3Angle);

        /* PID Error */
        SmartDashboard.putNumber("Joint 1 Error", arm.joint1PID.getPositionError());
        SmartDashboard.putNumber("Joint 2 Error", arm.joint2PID.getPositionError());

        SmartDashboard.putString("Arm State", String.valueOf(armState));
        SmartDashboard.putBoolean("Brake 1", Arm.joint1Brake.get() == Value.kReverse);
        SmartDashboard.putBoolean("Brake 2", Arm.joint2Brake.get() == Value.kReverse);
        
        /* In Position */
        SmartDashboard.putBoolean("Arm in Pos", arm.isArmInPosition());
        SmartDashboard.putBoolean("Is joint 1 at position", arm.isJoint1AtPosition());
        SmartDashboard.putBoolean("Is joint 2 at position", arm.isJoint2AtPosition());

        SmartDashboard.putBoolean("Auto Status", isAuto);
        
        SmartDashboard.putNumber("Sequence Step", currentSequence.currentIndex);
        SmartDashboard.putNumber("Num Steps", currentSequence.getLength());
        SmartDashboard.putBoolean("Sequence Finished", currentSequence.sequenceFinished());
        SmartDashboard.putNumber("Auto Step", autoStep);
        SmartDashboard.putBoolean("Timeout Override", timeoutOverride);

        /* Update Smart Dashboard with valid X & Y */
        SmartDashboard.putNumber("Valid x", currentArmPosition.x);
        SmartDashboard.putNumber("Valid y", currentArmPosition.y);

        /* Timeouts */
        SmartDashboard.putBoolean("J1 Timeout En", joint1TimeoutEnable);
        SmartDashboard.putNumber("J1 Timeout", joint1Timeout);
        SmartDashboard.putBoolean("J2 Timeout En", joint2TimeoutEnable);
        SmartDashboard.putNumber("J2 Timeout", joint2Timeout);


        // SmartDashboard.putNumber("Left Received Power", mg_leftDrive.get());
        // SmartDashboard.putNumber("Right Received Power", mg_rightDrive.get());

        SmartDashboard.putBoolean("Brakes Locked", lock);
        SmartDashboard.putNumber("IMU Axis", imu.getAngle());

    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        autoTimeoutTimer.reset();
        autoTimeoutTimer.start();
        isAuto = false;
        autoStep = 1;

        lock = false;

        arm.updateIntake(IntakeState.cubeClosed);

        m_driveLeft1.configOpenloopRamp(0.75);
        m_driveLeft2.configOpenloopRamp(0.75);
        m_driveRight1.configOpenloopRamp(0.75);
        m_driveRight2.configOpenloopRamp(0.75);

        Odometry.MAX_POWER = 0.35;

        autoSelected = autoChooser.getSelected();
        autoGamePiece = gamePieceChooser.getSelected();

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        double[] powers;
        r_robotDrive.feedWatchdog();
        if (autoStep == 1) {

            arm.updateArm(-1, 9, 90);

            odometry.straight(31.0);
            autoStep = 2;
            autoOdoCounter = 0;
            autoArmCounter = 0;

        } else if (autoStep == 2) {

            if (odometry.distanceTravelled > 20) {
                if (autoGamePiece == coneAuto) {
                    arm.updateArm(coneScoreFrontHigh.x, coneScoreFrontHigh.y, coneScoreFrontHigh.z);
                } else {
                    arm.updateArm(cubeScoreFrontHigh.x, cubeScoreFrontHigh.y + 2, cubeScoreFrontHigh.z - 20);
                }
            } else {
                arm.updateArm(-1, 9, 90);
            }

            powers = odometry.update();

            autoArmCounter++;
            mg_rightDrive.set(powers[0]);
            mg_leftDrive.set(powers[1]);

            if (Math.abs(Odometry.genPID.getPositionError()) < 6) autoOdoCounter++;
            if ((odometry.moveFinished() || autoOdoCounter > 125) && ((arm.isArmInPosition() && (Arm.joint1Brake.get() == Value.kReverse || Arm.joint2Brake.get() == Value.kReverse)) || autoArmCounter > 100)) {

                autoStep = 3;
                autoArmCounter = 0;
                
            }
            
        } else if (autoStep == 3) {

            if (autoGamePiece == coneAuto) {
                arm.updateArm(coneScoreFrontHigh.x, coneScoreFrontHigh.y, coneScoreFrontHigh.z);
            } else {
                arm.updateArm(cubeScoreFrontHigh);
            }

            autoArmCounter++;

            if (autoArmCounter > 125 || (arm.isArmInPosition())) {
                
                autoStep = 4;
                odometry.straight(-30);
                Odometry.MAX_POWER = 0.40;
            
            }

        } else if (autoStep == 4) {

            if (autoGamePiece == coneAuto) {
                arm.updateArm(coneScoreFrontHigh.x, coneScoreFrontHigh.y, coneScoreFrontHigh.z);
            } else {
                arm.updateArm(cubeScoreFrontHigh);
            }

            powers = odometry.update();

            mg_rightDrive.set(powers[0]);
            mg_leftDrive.set(powers[1]);

            if (odometry.moveFinished()) {

                autoStep = 5;
                autoIntakeCounter = 0;
                
            }

        } else if (autoStep == 5) {

            if (autoGamePiece == coneAuto) {
                arm.updateArm(coneScoreFrontHigh.x, coneScoreFrontHigh.y, coneScoreFrontHigh.z);
            } else {
                arm.updateArm(cubeScoreFrontHigh);
            }

            arm.updateIntake(IntakeState.cubeOpen);
            autoIntakeCounter++;

            if (autoIntakeCounter > 25) {
                
                autoArmCounter = 0;

                autoStep = 6;
                Odometry.MAX_POWER = 0.45;

                switch (autoSelected) {
                    case (defaultAuto):
                        odometry.straight(0);
                        break;
                    case (rightAuto):
                        odometry.straight(15);
                        break;
                    case (leftAuto):
                        odometry.straight(159);
                        break;
                    case (middleAuto):
                        odometry.straight(10);
                        break;
                    case (middleChargeAuto):
                        odometry.straight(97.0);
                        break;
                }

            }

        } else if (autoStep == 6) {

            powers = odometry.update();

            m_driveLeft1.setNeutralMode(NeutralMode.Brake);
            m_driveLeft2.setNeutralMode(NeutralMode.Brake);
            m_driveRight1.setNeutralMode(NeutralMode.Brake);
            m_driveRight2.setNeutralMode(NeutralMode.Brake);

            autoArmCounter++;
            if (autoArmCounter > 25) arm.updateArm(0, 9, 90);

            mg_rightDrive.set(powers[0]);
            mg_leftDrive.set(powers[1]);


            if (odometry.moveFinished()) {

                autoStep = 7;
                autoIntakeCounter = 0;
                
                lock = true;

            }

        } else {

            arm.updateArm(0, 9, 90);
        
            if (autoSelected == middleChargeAuto) {

                powers = odometry.balance(imu.getAngle(), imu.getRate());

            } else {

                powers = odometry.update();

            }

            m_driveLeft1.setNeutralMode(NeutralMode.Brake);
            m_driveLeft2.setNeutralMode(NeutralMode.Brake);
            m_driveRight1.setNeutralMode(NeutralMode.Brake);
            m_driveRight2.setNeutralMode(NeutralMode.Brake);

            mg_rightDrive.set(powers[0]);
            mg_leftDrive.set(powers[1]);
            
        }

        /* Just for extra protection */
        if (autoTimeoutTimer.get() > 14.5) {

            Arm.joint1Brake.set(Value.kReverse);
            Arm.joint2Brake.set(Value.kReverse);

            // arm.setJoint1(0);
            // arm.setJoint2(0);
            // arm.setJoint3(0);

            // m_driveLeft1.setNeutralMode(NeutralMode.Brake);
            // m_driveLeft2.setNeutralMode(NeutralMode.Brake);
            // m_driveRight1.setNeutralMode(NeutralMode.Brake);
            // m_driveRight2.setNeutralMode(NeutralMode.Brake);

            // mg_rightDrive.set(0);
            // mg_leftDrive.set(0);

            // autoTimeoutTimer.stop();
        }

    

    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {

        lock = false;

        m_driveLeft1.setNeutralMode(NeutralMode.Coast);
        m_driveLeft2.setNeutralMode(NeutralMode.Coast);
        m_driveRight1.setNeutralMode(NeutralMode.Coast);
        m_driveRight2.setNeutralMode(NeutralMode.Coast);

        isAuto = false;
        // currentArmPosition.setCoordinates(0, 9, 90);
        double[] armCoordinates = Arm.getArmCoordinates(arm.joint1CurrentPosition(), arm.joint2CurrentPosition());
        currentArmPosition.setCoordinates(armCoordinates[0], armCoordinates[1], arm.joint3CurrentPosition());
        arm.joint1PID.reset();
        arm.joint2PID.reset();
        arm.joint3PID.reset();

        m_driveLeft1.configOpenloopRamp(0.25);
        m_driveLeft2.configOpenloopRamp(0.25);
        m_driveRight1.configOpenloopRamp(0.25);
        m_driveRight2.configOpenloopRamp(0.25);

    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {

        /**
         * CONTROLS:
         * Driver 1 Closes and opens claw, Driver one controls what side of the claw is
         * used for pickup
         * Driver 1 (and 2?) controls joint 3
         *
         * Driver 2 controls arm automatic positioning and manual adjustments (all
         * through inverse kinematics)
         * Driver 2 controls initial joint 3 position
         */

        // Arm Position controls
        // TODO if robot drive fast for extended period of time (a few seconds?) set arm
        // TODO to position within bumpers

        // Uncomment for driving

        /* Claw States */

        /* Switch between Cone and Cube */
        if (farmSim2.getRawButton(6)) {
            coneControl = true;
        } else if (farmSim2.getRawButton(7)) {
            coneControl = false;
        }

        /* Home Control */
        if (farmSim1.getRawButton(4))
            armState = ArmState.goFrontHome;
        if (farmSim1.getRawButton(5))
            armState = ArmState.goBackHome;
        if (farmSim2.getRawButtonPressed(5))
            armState = ArmState.goHome;

        if (farmSim2.getRawButtonPressed(3))
            armState = ArmState.goConePickupOnSideFront;
        if (farmSim2.getRawButtonPressed(4))
            armState = ArmState.goConePickupOnSideBack;
        /* Scoring Positions */
        if (coneControl) {
            /* Claw Control */
            // if (c_controller.getXButtonPressed()) intakeState =
            // IntakeState.coneIntakeBack;
            // if (c_controller.getBButtonPressed()) intakeState =
            // IntakeState.coneIntakeForward;
            // if (c_controller.getAButtonPressed()) intakeState = IntakeState.coneClosed;

            if (c_controller.getXButtonPressed() || c_controller.getBButtonPressed())
                intakeState = IntakeState.cubeOpen;
            if (c_controller.getAButtonPressed())
                intakeState = IntakeState.cubeClosed;

            /* Pickup Positions */
            if (farmSim2.getRawButtonPressed(2))
                armState = ArmState.goConePickupBackGround;
            if (farmSim2.getRawButtonPressed(1))
                armState = ArmState.goConePickupFrontGround;

            if (farmSim1.getRawButtonPressed(10))
                armState = ArmState.goConePickupBackShelf;
            if (farmSim1.getRawButtonPressed(9))
                armState = ArmState.goConePickupFrontShelf;

            /* Scoring Positions */
            if (farmSim2.getRawButtonPressed(13))
                armState = ArmState.goConeScoreBackLow;
            if (farmSim1.getRawButtonPressed(7))
                armState = ArmState.goConeScoreBackMiddle;
            if (farmSim1.getRawButtonPressed(2))
                armState = ArmState.goConeScoreBackHigh;

            if (farmSim2.getRawButtonPressed(11))
                armState = ArmState.goConeScoreFrontLow;
            if (farmSim1.getRawButtonPressed(6))
                armState = ArmState.goConeScoreFrontMiddle;
            if (farmSim1.getRawButtonPressed(1))
                armState = ArmState.goConeScoreFrontHigh;
        } else {
            /* Claw Control */
            if (c_controller.getXButtonPressed() || c_controller.getBButtonPressed())
                intakeState = IntakeState.cubeOpen;
            if (c_controller.getAButtonPressed())
                intakeState = IntakeState.cubeClosed;

            /* Pickup Positions */
            if (farmSim2.getRawButtonPressed(2))
                armState = ArmState.goCubePickupBackGround;
            if (farmSim2.getRawButtonPressed(1))
                armState = ArmState.goCubePickupFrontGround;

            if (farmSim1.getRawButtonPressed(10))
                armState = ArmState.goCubePickupBackShelf;
            if (farmSim1.getRawButtonPressed(9))
                armState = ArmState.goCubePickupFrontShelf;

            /* Scoring Positions */
            if (farmSim2.getRawButtonPressed(12))
                armState = ArmState.goCubeScoreBackLow;
            if (farmSim1.getRawButtonPressed(7))
                armState = ArmState.goCubeScoreBackMiddle;
            if (farmSim1.getRawButtonPressed(2))
                armState = ArmState.goCubeScoreBackHigh;

            if (farmSim2.getRawButtonPressed(11))
                armState = ArmState.goCubeScoreFrontLow;
            if (farmSim1.getRawButtonPressed(6))
                armState = ArmState.goCubeScoreFrontMiddle;
            if (farmSim1.getRawButtonPressed(1))
                armState = ArmState.goCubeScoreFrontHigh;
        }

        /* Set Claw state */
        arm.updateIntake(intakeState);

        if (farmSim2.getRawButtonPressed(12))
            lock = !lock;

        /* Enable Inverse Kinematic PID Control of arm */
        if (c_controller.getBackButtonPressed()) {
            // currentArmPosition.setCoordinates(0, 9, 0);
            // currentArmPosition = new ArmPosition(home);
            armState = ArmState.home;
            isAuto = !isAuto;
        }

        /* Command arm to go to different positions */
        switch (armState) {

            case goConePickupOnSideFront:
                desiredArmPosition = conePickupOnSideFront;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupOnSideFront;
                break;

            case goConePickupOnSideBack:
                desiredArmPosition = conePickupOnSideBack;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupOnSideBack;
                break;

            case goConePickupFrontGround:
                desiredArmPosition = conePickupFrontGround;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupFrontGround;
                break;

            case goConePickupBackGround:
                desiredArmPosition = conePickupBackGround;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupBackGround;
                break;

            case goConePickupFrontShelf:
                desiredArmPosition = conePickupFrontShelf;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupFrontShelf;
                break;

            case goConePickupBackShelf:
                desiredArmPosition = conePickupBackShelf;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.conePickupBackShelf;
                break;

            case goCubePickupFrontGround:
                desiredArmPosition = cubePickupFrontGround;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubePickupFrontGround;
                break;

            case goCubePickupBackGround:
                desiredArmPosition = cubePickupBackGround;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubePickupBackGround;
                break;

            case goCubePickupFrontShelf:
                desiredArmPosition = cubePickupFrontShelf;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubePickupFrontShelf;
                break;

            case goCubePickupBackShelf:
                desiredArmPosition = cubePickupBackShelf;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubePickupBackShelf;
                break;

            case goConeScoreFrontLow:
                desiredArmPosition = coneScoreFrontLow;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreFrontLow;
                break;

            case goConeScoreFrontMiddle:
                desiredArmPosition = coneScoreFrontMiddle;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreFrontMiddle;
                break;

            case goConeScoreFrontHigh:
                desiredArmPosition = coneScoreFrontHigh;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreFrontHigh;
                break;

            case goConeScoreBackLow:
                desiredArmPosition = coneScoreBackLow;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreBackLow;
                break;

            case goConeScoreBackMiddle:
                desiredArmPosition = coneScoreBackMiddle;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreBackMiddle;
                break;

            case goConeScoreBackHigh:
                desiredArmPosition = coneScoreBackHigh;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.coneScoreBackHigh;
                break;

            case goCubeScoreFrontLow:
                desiredArmPosition = cubeScoreFrontLow;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreFrontLow;
                break;

            case goCubeScoreFrontMiddle:
                desiredArmPosition = cubeScoreFrontMiddle;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreFrontMiddle;
                break;

            case goCubeScoreFrontHigh:
                desiredArmPosition = cubeScoreFrontHigh;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreFrontHigh;
                break;

            case goCubeScoreBackLow:
                desiredArmPosition = cubeScoreBackLow;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreBackLow;
                break;

            case goCubeScoreBackMiddle:
                desiredArmPosition = cubeScoreBackMiddle;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreBackMiddle;
                break;

            case goCubeScoreBackHigh:
                desiredArmPosition = cubeScoreBackHigh;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.cubeScoreBackHigh;
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

            case goBackHome:
                desiredArmPosition = homeBack;
                Arm.armControlState = ArmControlState.newPath;
                armState = ArmState.backHome;
                break;

            default:
                break;
        }

        /* If Inverse Kinematic PID Control is enabled */
        if (isAuto) {
            double fbSpeed = 0;

            if (c_controller.getStartButtonPressed()) {

                brake = !brake;

                if (brake) {
    
                    m_driveLeft1.setNeutralMode(NeutralMode.Brake);
                    m_driveLeft2.setNeutralMode(NeutralMode.Brake);
                    m_driveRight1.setNeutralMode(NeutralMode.Brake);
                    m_driveRight2.setNeutralMode(NeutralMode.Brake);
                
                } else {
    
                    m_driveLeft1.setNeutralMode(NeutralMode.Coast);
                    m_driveLeft2.setNeutralMode(NeutralMode.Coast);
                    m_driveRight1.setNeutralMode(NeutralMode.Coast);
                    m_driveRight2.setNeutralMode(NeutralMode.Coast);
                
                }

            }

            if (c_controller.getPOV() == 0)
                fbSpeed = -0.45;
            else if (c_controller.getPOV() == 180)
                fbSpeed = 0.45;
            else
                fbSpeed = c_controller.getLeftY();

            r_robotDrive.arcadeDrive(fbSpeed, 0.75 * c_controller.getRightX());

            /*************************************************************************************
             * START JOINT 3 MANUAL CONTROL
             *************************************************************************************/
            /* If manual inputs are present */
            if (Math.abs(farmSim1.getY()) > 0.1 || Math.abs(farmSim1.getX()) > 0.1 ||
                    c_controller.getPOV() == 90 || c_controller.getPOV() == 270) {
                /* Put the arm into manual mode */
                armState = ArmState.manual;
                Arm.armControlState = ArmControlState.manual;
            }

            /* Rotate Joint 3 towards back of robot */
            if (c_controller.getPOV() == 90) {
                currentArmPosition.incrementZ(-3);
            }
            /* Rotate Joint 3 towards front of robot */
            else if (c_controller.getPOV() == 270) {
                currentArmPosition.incrementZ(3);
            }

            /* If Joint 3 angle is over 336 degrees reset to 336 */
            if (currentArmPosition.z > 336) {
                currentArmPosition.z = 336;
            }
            /* If Joint 3 angle is under -69 degrees reset to -69 */
            else if (currentArmPosition.z < -69) {
                currentArmPosition.z = -69;
            }

            /*************************************************************************************
             * END JOINT 3 MANUAL CONTROL
             *************************************************************************************/

            /*************************************************************************************
             * START X & Y MANUAL CONTROL
             *************************************************************************************/
            /* Increment X & Y */
            currentArmPosition.incrementX(deadZone(farmSim1.getX() + farmSim2.getX()) * -0.25);
            currentArmPosition.incrementY(deadZone(farmSim1.getY() + farmSim2.getY()) * -0.25);

            if (armState == ArmState.manual) {
                /* If current position is within frame perimeter */
                if (currentArmPosition.returnSide() == 0) {
                    /* Force Y to be at 9 in to avoid eratic movement */
                    currentArmPosition.y = 9;
                    // TODO: Remove setting Joint 3 to 0 once Arm Path Planning is working
                    currentArmPosition.z = 90;
                }
            }

            /* Arm Checks */
            // SmartDashboard.putBoolean("Check 1", (Arm.c <= (Arm.a + Arm.b)));
            // SmartDashboard.putBoolean("Check 2", (Arm.c >= (Arm.b - Arm.a)));
            // SmartDashboard.putBoolean("Check 3",
            //         ((Math.toDegrees(Arm.joint1Angle) <= 60) && (Math.toDegrees(Arm.joint1Angle) >= -60)));
            // SmartDashboard.putBoolean("Check 4",
            //         ((Math.toDegrees(Arm.joint2Angle) <= 150) && (Math.toDegrees(Arm.joint2Angle) >= -150)));
            // SmartDashboard.putBoolean("Check 5", (currentArmPosition.y <= 70.5));
            // SmartDashboard.putBoolean("Check 6", (currentArmPosition.x <= 63.0 && currentArmPosition.x >= -63.0));

            /* If attempted arm position is invalid during manual control */
            if (!arm.isArmPositionValid(currentArmPosition.x, currentArmPosition.y) && armState == ArmState.manual) {
                /* Reset currentArmPosition to last know valid position */
                currentArmPosition.x = Arm.lastValidX;
                currentArmPosition.y = Arm.lastValidY;
            }
            /* If attempted arm position is valid, update last know valid position */
            else if (armState == ArmState.manual) {
                Arm.lastValidX = currentArmPosition.x;
                Arm.lastValidY = currentArmPosition.y;
            }

            
            /*************************************************************************************
             * END X & Y MANUAL CONTROL
             *************************************************************************************/

            /*************************************************************************************
             * START ARM STATE CONTROL
             *************************************************************************************/
            if (Arm.armControlState == ArmControlState.manual) {
                /*
                 * If arm is controller manually, the current position is updated above, the arm
                 * will be told to run to updated position in the arm.updateArm() call below.
                 */
            }
            /* If a new desiredArmPositon has been requested */
            else if (Arm.armControlState == ArmControlState.newPath) {

                /* Put arm in runningPath state */
                Arm.armControlState = ArmControlState.runningPath;
                /*
                 * Create a new sequence to move from currentArmPosition to desiredArmPosition
                 */
                currentSequence = pathPlanner.planPath(new ArmPosition(currentArmPosition),
                                                       new ArmPosition(desiredArmPosition));

                joint1TimeoutEnable = true;
                joint2TimeoutEnable = true;
                joint1Timeout = 0;
                joint2Timeout = 0;
                timeoutOverride = false;

                currentArmPosition = new ArmPosition(currentSequence.nextPosition());
            }

            /* If the arm is in the middle of a sequence */
            else if (Arm.armControlState == ArmControlState.runningPath) {

                /* Next position and path finished */
                if ((arm.isArmInPosition() || (timeoutOverride && arm.isJoint3AtPosition())) && !currentSequence.sequenceFinished()) {
                    timeoutOverride = false;
                    /* set currentArmPosition to next position in the sequence */
                    currentArmPosition = new ArmPosition(currentSequence.nextPosition());
                    joint1TimeoutEnable = true;
                    joint2TimeoutEnable = true;

                    /* mighta been it */
                    joint1Timeout = 0;
                    joint2Timeout = 0;
                }
                /* If the arm is in position and the sequence is finished */
                else if (arm.isArmInPosition() && currentSequence.sequenceFinished()) {
                    /* Put the arm in a completed path state */
                    Arm.armControlState = ArmControlState.completedPath;
                }

                /* Arm position timeouts */
                if ((!arm.isJoint1AtPosition()) && joint1TimeoutEnable && (joint1Timeout <= joint1TimeoutLimit)) {
                    joint1Timeout++;
                }

                if ((!arm.isJoint2AtPosition()) && joint2TimeoutEnable && (joint2Timeout <= joint2TimeoutLimit)) {
                    joint2Timeout++;
                }

                /* Service timeouts */
                if ( (joint1Timeout > joint1TimeoutLimit && joint2Timeout > joint2TimeoutLimit) ||
                     ( joint1Timeout > joint1TimeoutLimit && arm.isJoint2AtPosition()) || 
                     ( joint2Timeout > joint2TimeoutLimit && arm.isJoint1AtPosition())) 
                {
                    timeoutOverride = true;

                    /* Reset timeouts */
                    joint1Timeout = 0;
                    joint1TimeoutEnable = false;
                    joint2Timeout = 0;
                    joint2TimeoutEnable = false;
                }
            }

            /* Update Arm PID calculation and motor set speeds */
            arm.updateArm(currentArmPosition);

            /*************************************************************************************
             * END ARM STATE CONTROL
             *************************************************************************************/
        }
        /* Inverse Kinematic PID Control is disabled */
        else {

            r_robotDrive.arcadeDrive(0, 0);

            Arm.joint1Brake.set(Value.kForward);
            Arm.joint2Brake.set(Value.kForward);

            arm.setJoint1(deadZone(c_controller.getLeftX()) * 0.1);
            arm.setJoint2(deadZone(c_controller.getRightX()) * 0.075);

        }

    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        // if (bbRight.getRawButton(1))
        //     leftDriveEnc.setQuadraturePosition(0, 0);
        // if (bbRight.getRawButton(4))
            // rightDriveEnc.setQuadraturePosition(0, 0);

    }

    /* Apply a fixed dead zone to an input */
    public double deadZone(double input) {
        // TODO: Fix to scale input after deadzone

        if (input < 0.1 && input > -0.1)
            return 0;
        else
            return input;

    }

}

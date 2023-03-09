package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Arm {

    /* Arm Motor Controllers */
    static CANSparkMax m_Joint1_1 = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    static CANSparkMax m_Joint1_2 = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    static CANSparkMax m_Joint2 = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    static MotorControllerGroup mg_Joint1 = new MotorControllerGroup(m_Joint1_1, m_Joint1_2);

    static CANSparkMax m_Joint3 = new CANSparkMax(13, MotorType.kBrushless);

    // Limit Switches for joint 1
    DigitalInput ls_joint1PosLimit = new DigitalInput(0);
    DigitalInput ls_joint1NegLimit = new DigitalInput(1);

    WPI_TalonSRX joint2EncoderTalon;
    WPI_TalonSRX joint1EncoderTalon;

    public static PneumaticsControlModule pmc = new PneumaticsControlModule(1);
    public static DoubleSolenoid joint1Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
    public static DoubleSolenoid joint2Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3);
    public static DoubleSolenoid intake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5);
    public static DoubleSolenoid coneSwitch = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 6, 7);

    public enum ArmState {
        /* Cone Pickups */
        goConePickupFrontGround, conePickupFrontGround,
        goConePickupBackGround, conePickupBackGround,
        goConePickupFrontShelf, conePickupFrontShelf,
        goConePickupBackShelf , conePickupBackShelf,
        /* Cube Pickups */
        goCubePickupFrontGround , cubePickupFrontGround,
        goCubePickupBackGround , cubePickupBackGround,
        goCubePickupFrontShelf , cubePickupFrontShelf,
        goCubePickupBackShelf , cubePickupBackShelf,
        /* Cone Score */
        goConeScoreFrontLow, coneScoreFrontLow,
        goConeScoreFrontMiddle, coneScoreFrontMiddle,
        goConeScoreFrontHigh, coneScoreFrontHigh,
        goConeScoreBackLow, coneScoreBackLow,
        goConeScoreBackMiddle, coneScoreBackMiddle,
        goConeScoreBackHigh, coneScoreBackHigh,
        /* Cube Score */
        goCubeScoreFrontLow, cubeScoreFrontLow,
        goCubeScoreFrontMiddle, cubeScoreFrontMiddle,
        goCubeScoreFrontHigh, cubeScoreFrontHigh,
        goCubeScoreBackLow, cubeScoreBackLow,
        goCubeScoreBackMiddle, cubeScoreBackMiddle,
        goCubeScoreBackHigh, cubeScoreBackHigh,
        /* Homes */
        goFrontHome, frontHome, 
        goBackHome, backHome, 
        goHome, home, 
        manual

    }

    public enum ArmControlState {

        manual,
        newPath,
        runningPath,
        completedPath

    }

    public static ArmControlState armControlState = ArmControlState.manual;

    int numArmStates = 12;

    double[][] armSetPositions = new double[numArmStates][3];

    public enum IntakeState {

        coneIntakeForward, coneIntakeBack,
        cubeOpen, cubeClosed,
        coneClosed

    }

    double joint1kP_base = 0.0045;
    double joint1_physicsMult = 0.0019;
    double arm2Influence = 0.54;
    double joint1kI = 0.000;
    double joint1kD = 0.0;
    PIDController joint1PID = new PIDController(joint1kP_base, joint1kI, joint1kD);

    double joint2kP_base = 0.0065;
    double joint2_physicsMult = 0.002;
    double joint2kI = 0.0000;
    double joint2kD = 0.000;
    PIDController joint2PID = new PIDController(joint2kP_base, joint2kI, joint2kD);


    double joint3kP = 0.004;
    double joint3kI = 0;
    double joint3kD = 0.001;


    PIDController joint3PID = new PIDController(joint3kP, joint3kI, joint3kD);

    public Arm(WPI_TalonSRX joint1EncoderTalon, WPI_TalonSRX joint2EncoderTalon) {
        
        /* Joint 1 and 2 Encoders */
        this.joint1EncoderTalon = joint1EncoderTalon;
        this.joint2EncoderTalon = joint2EncoderTalon;

        /* Joint 1 Ramp Rate */
        m_Joint1_1.setOpenLoopRampRate(3);
        m_Joint1_2.setOpenLoopRampRate(3);
        /* Joint 2 and 3 Ramp Rate */
        m_Joint2.setOpenLoopRampRate(3);
        m_Joint3.setOpenLoopRampRate(3);

        /* PID Tolerance */
        joint1PID.setTolerance(2);
        joint2PID.setTolerance(3);
        joint3PID.setTolerance(3);

    }

    int joint1AtPositionPersistence, joint2AtPositionPersistence;

    public static double joint1Angle, joint2Angle = 0;
    public static double c = 0;
    public static double angleY, angleA = 0;

    public static double a = 33;
    public static double b = 42;

    public static double joint2Calc;

    public void setArmCoordinates(ArmPosition position) {

        setArmCoordinates(position.x, position.y);

    }

    public void setArmCoordinates(double x, double y) {

        c = Math.sqrt((x * x) + (y * y));
        angleY = Math.asin(y/c);
        double ACalc = ((b * b) + (c * c) - (a * a)) / (2 * c * b); // calculate cosine angle A
        angleA = Math.acos(ACalc); // Find angle A
        joint1Angle = (Math.PI / 2) - (angleY + angleA);

        double cosineJoint2 = ((a * a) + (b * b) - (c * c)) / (2 * a * b); // calculate cosine Joint2 or angle C
        joint2Angle = Math.acos(cosineJoint2);

        if ((x < 0.5 && x > -0.5) && (y < 9.5)) {

            joint1Angle = 0;
            joint2Angle = 0;

        }

        if (x < 0) {

            joint1Angle = -joint1Angle;
            joint2Angle = -joint2Angle;

        }

        // System.out.println("Joint 1 Angle: " + Math.toDegrees(joint1Angle));

        // System.out.println("Joint 2 Angle: " + Math.toDegrees(joint2Angle));

        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));

    }

    public static double[] getArmCoordinates(double joint1Angle, double joint2Angle) {

        double armCoordinates[] = new double[2];

        double x, y = 0;

        double x1, y1 = 0;
        double x2, y2 = 0;

        x1 = Math.sin(Math.toRadians(joint1Angle)) * b;
        y1 = Math.cos(Math.toRadians(joint1Angle)) * b;

        x2 = Math.sin(Math.toRadians(joint2Angle)) * a;
        y2 = -Math.cos(Math.toRadians(joint2Angle)) * a;

        x = x1 + x2;
        y = y1 + y2;

        armCoordinates[0] = x;
        armCoordinates[1] = y;
        // System.out.println("x1: " + x1);
        // System.out.println("y1: " + y1);
        // System.out.println("x2: " + x2);
        // System.out.println("y2: " + y2);


        //  System.out.println("Current x: " + x);

        //  System.out.println("Current y: " + y);

       SmartDashboard.putNumber("Current x", x);
       SmartDashboard.putNumber("Current y", y);

       return armCoordinates;

    }

    public boolean isArmPositionValid(double x, double y) {

        setArmCoordinates(x, y);

        return 
        (c <= (a + b)) && // not longer than physically possible
        (c >= (b - a)) && // not shorter than physically possible
        ((Math.toDegrees(joint1Angle) <= 60) && (Math.toDegrees(joint1Angle) >= -60)) && // joint 1 isn't outside of limits
        ((Math.toDegrees(joint2Angle) <= 150) && (Math.toDegrees(joint2Angle) >= -150)) && // joint 2 isn't outside of limits
        (y <= 70.5) && // doesn't exceed height limit
        (x <= 63.0 && x >= -63.0); // doesn't exceed 48 inches from frame

    }

    // double x, y = 0;
    double joint1Pos, joint2Pos, joint2SetPoint = 0;
    double joint1Speed, joint2Speed = 0;
    public static double lastValidX = 0;
    public static double lastValidY = 0;

    public void updateArm(ArmPosition position) {

        double x = position.x;
        double y = position.y;
        double z = position.z;

        updateArm(x, y, z);

    }

    public void updateArm(double x, double y, double z) {

        if (isArmPositionValid(x, y)) {

            setArmCoordinates(x, y);
            if ((x < 0.5 && x > -0.5) && y < 9.5) {

                joint1Angle = 0;
                joint2Angle = 0;

            }

            joint2SetPoint = joint2Angle - joint1Angle;
            joint1PID.setSetpoint(Math.toDegrees(joint1Angle));
            joint2PID.setSetpoint(Math.toDegrees(joint2SetPoint));

            joint1Speed = joint1PID.calculate(joint1CurrentPosition());
            joint2Speed = joint2PID.calculate(joint2CurrentPosition());

            // dynamicI(joint1PID.getPositionError(), joint2PID.getPositionError());
            dynamicPower(joint1CurrentPosition(), joint1Speed, joint2CurrentPosition(), joint2Speed);
            
            joint3AutoTest(z);

            if (!isJoint1AtPosition()) {
                joint1Brake.set(DoubleSolenoid.Value.kForward);
                joint1AtPositionPersistence = 0;
            } else {

                
                if (joint1AtPositionPersistence > 25) {
                    joint1Brake.set(DoubleSolenoid.Value.kReverse);
                } else {

                    joint1AtPositionPersistence++;

                }

            }

            if (!isJoint2AtPosition()) {
                joint2Brake.set(DoubleSolenoid.Value.kForward);
                joint2AtPositionPersistence = 0;
            } else {

                if (joint2AtPositionPersistence > 50) {
                    joint2Brake.set(DoubleSolenoid.Value.kReverse);
                } else {

                    joint2AtPositionPersistence++;

                }

            }
        
        }

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));
        SmartDashboard.putNumber("joint2Setpoint", Math.toDegrees(joint2SetPoint));
        SmartDashboard.putNumber("Joint 1 Power", mg_Joint1.get());
        SmartDashboard.putNumber("Joint 2 Power", m_Joint2.get());
        SmartDashboard.putNumber("Joint 1 Error", joint1PID.getPositionError());
        SmartDashboard.putNumber("Joint 2 Error", joint2PID.getPositionError());

        //  joint1Pos = joint1Enc.get() * 2 * Math.PI;
//    joint2Pos = joint2Enc.get() * 2 * Math.PI;

//     m_Joint2.set(joint2Speed);

    }

    double joint3Speed = 0;

    public void dynamicI(double joint1Error, double joint2Error) {

        if (Math.abs(joint1Error) < 3) {

            // joint1PID.setI(0);


        } else if (Math.abs(joint1Error) < 4) {

            joint1PID.setI(joint1kI);

        } else if (Math.abs(joint1Error) < 6) {

            joint1PID.setI(joint1kI * 0.7);

        } else {

            joint1PID.setI(0);

        }

        // if (Math.abs(joint2Error) < 5) {

        //     joint2PID.setI(0);

        // } else if (Math.abs(joint2Error) < 15) {

        //     joint2PID.setI(joint2kI);

        // } else if (Math.abs(joint2Error) < 20) {

        //     // joint2PID.setI(joint2kI * 0.4);
        //     joint2PID.setI(0);

        // } else {

        //     joint2PID.setI(0);

        // }

    }

    public void dynamicPower(double joint1Angle, double joint1Power, double joint2Angle, double joint2Power) {

        double joint1Radians = Math.toRadians(joint1Angle);
        double joint2Radians = Math.toRadians(joint2Angle);

        /* Arm parameters */
        double arm1Weight = 8;
        double arm1Length = 42.0;

        double arm2Weight = 4;
        double arm2Length = 33.0;

        double clawWeight = 5.7;

        /* Physics calculations */
        double arm1XDist = arm1Length * Math.sin(joint1Radians);
        double arm1Moment = arm1XDist / 2 * arm1Weight;

        double arm2XDist = arm2Length * Math.sin(joint2Radians);
        double arm2Moment = (arm2XDist / 2) * arm2Weight;
        double arm2MomentJoint1 = ( (arm2XDist / 2) + arm1XDist) * arm2Weight;

        double clawMoment = arm2XDist * clawWeight;
        double clawMomentJoint1 = (arm1XDist + arm2XDist) * clawWeight;

        double joint1Moment = (arm1Moment + (arm2MomentJoint1 + clawMomentJoint1) * arm2Influence) / 12;
        double joint2Moment = (arm2Moment + clawMoment) / 12;

        double joint1Added = 0;
        double joint2Added = 0;

        SmartDashboard.putNumber("Joint 1 Moment", joint1Moment);
        SmartDashboard.putNumber("Joint 2 Moment", joint2Moment);

        SmartDashboard.putNumber("arm1XDist Dist", arm1XDist);
        SmartDashboard.putNumber("arm2XDist Dist", arm2XDist);

        joint1Added = -joint1Moment * this.joint1_physicsMult;
        joint2Added = joint2Moment * this.joint2_physicsMult;
        
        SmartDashboard.putNumber("Joint 1 Added P", joint1Added);
        SmartDashboard.putNumber("Joint 2 Added P", joint2Added);

        /* If arm is not in position, set motor power to physics offset + PID request,
         * otherwise just use the physics offset. The physics offset will be tuned to
         * just barely stall the motor. */        
        if (!isJoint1AtPosition()) {
            setJoint1(joint1Added + joint1Power);
        } else {
            setJoint1(joint1Added);

        }

        if (!isJoint2AtPosition()) {
            setJoint2(joint2Added + joint2Power);
        } else {
            setJoint2(joint2Added);

        }
    }

    public static double joint3Angle;

    public void joint3AutoTest(double angle) {

        joint3Angle = angle;
        joint3PID.setSetpoint(angle);
        joint3Speed = joint3PID.calculate(joint3CurrentPosition());
        setJoint3(joint3Speed);

    }

    public void setJoint1(double power) {

        if ((power > 0 && !ls_joint1PosLimit.get()) || (power < 0 && !ls_joint1NegLimit.get())) {

            joint1Speed = 0;

        } else if (power > 0.3) {

            joint1Speed = 0.3;

        } else if (power < -0.3) {

            joint1Speed = -0.3;

        } else {

            joint1Speed = power;

        }

        // if (joint1Brake.get() == Value.kReverse)
        // { joint1Speed = 0; }

        mg_Joint1.set(joint1Speed);
    }

    public void setJoint2(double power) {

        if (power > 0.25) {

            joint2Speed = 0.25;

        } else if (power < -0.25) {

            joint2Speed = -0.25;

        } else {

            joint2Speed = power;

        }
        
        /* Prevent joint movement while the arm is down */
        // if (Math.abs(joint1CurrentPosition()) > 55)
        // { 
        //     joint2Speed = 0; 
        // }

        m_Joint2.set(joint2Speed);
    }

    public void setJoint3(double power) {

        if (power > 0.9) {

            joint3Speed = 0.9;

        } else if (power < -0.9) {

            joint3Speed = -0.9;

        } else {

            joint3Speed = power;

        }

        m_Joint3.set(joint3Speed);
        // SmartDashboard.putNumber("Joint 2 Angle", joint2CurrentPosition());

    }

    public double joint1CurrentPosition() {

        // return ((Robot.Joint2Enc.getPulseWidthPosition() - 351.0) / 4096.0 * 360.0);
        return Robot.Joint1Enc.getPulseWidthPosition() / 11.37 - 223;

    }

    public double joint2CurrentPosition() {

        // return ((Robot.Joint2Enc.getPulseWidthPosition() - 351.0) / 4096.0 * 360.0);
        return Robot.Joint2Enc.getPulseWidthPosition() / 11.37 - 268;

    }

    public double joint3CurrentPosition() {

        return (m_Joint3.getEncoder().getPosition()) / 60 * 200.0 * 1.125;

    }

    public boolean isJoint1AtPosition() {

        return (Math.abs(joint1PID.getPositionError()) < 3);

    }

    public boolean isJoint2AtPosition() {

        return (Math.abs(joint2PID.getPositionError()) < 3);

    }

    public boolean isJoint3AtPosition() {

        return (Math.abs(joint3PID.getPositionError()) < 5);

    }

    public boolean isArmInPosition() {

        return isJoint1AtPosition() && isJoint2AtPosition() && isJoint3AtPosition();

    }

    public void updateIntake(IntakeState intakeState) {

        switch (intakeState) {

            case cubeOpen:
            intake.set(Value.kReverse);
            coneSwitch.set(Value.kForward);
            break;

            case coneIntakeForward:
            intake.set(Value.kForward);
            coneSwitch.set(Value.kForward);
            break;

            case coneIntakeBack:
            intake.set(Value.kForward);
            coneSwitch.set(Value.kReverse);
            break;

            case cubeClosed:
            intake.set(Value.kForward);
            break;
            
            case coneClosed:
            intake.set(Value.kReverse);
            coneSwitch.set(Value.kForward);
            break;

        }

    }

}

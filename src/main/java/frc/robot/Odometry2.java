package frc.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry {


    /* Drive Base Motor Controllers */
    public static MotorControllerGroup rightDrive;
    public static MotorControllerGroup leftDrive;
    DifferentialDrive robotDrive;

    /* Drive Encoders */
    public static SensorCollection leftEncoder;
    public static SensorCollection rightEncoder;

    // constants for inch calibration
    double countsPerRev = 4096;
    double wheelDiameter = 6.0; // inches
    double wheelCircumference = wheelDiameter * Math.PI;
    double gearRatio = 1.0;

    // PID constants
    public static double P_gen_forwardback = 0.5;
    public static double P_gen_turn = 0.03;
    public static double P_differential = 0.1;

    // Speed constants
    public static double MAX_POWER = 0.05;
    public static double MIN_POWER_STRAIGHT = 0;
    public static double MIN_POWER_TURN = 0;
        
    double rightOffset = 0;
    double leftOffset = 0;
    double genPower = 0;
    double diffPower = 0;

    double distanceTravelled = 0;
    double error = 0;
    double differentialError = 0;

    double rightTravelled, leftTravelled = 0;

    public static boolean destinationReached = false;

    public Odometry(MotorControllerGroup leftDrive,
                    MotorControllerGroup rightDrive,
                    DifferentialDrive robotDrive,
                    SensorCollection leftEncoder, SensorCollection rightEncoder) {
        
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        
        this.robotDrive = robotDrive;

        leftDrive.setInverted(true);

        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

    }

    public double toInches(double encoderCount) {

        double inches = encoderCount / this.countsPerRev * this.gearRatio * this.wheelCircumference;

        return inches;
    }

    public void zeroEncoders() {

        leftEncoder.setQuadraturePosition(0, 0);
        rightEncoder.setQuadraturePosition(0, 0);

    }

    public void forward(double INCHES) {
        
        destinationReached = this.distanceTravelled > INCHES;
        SmartDashboard.putNumber("Distance Traveled", this.distanceTravelled);
        SmartDashboard.putBoolean("Dist Reached", destinationReached);
        SmartDashboard.putNumber("Dist Error", this.error);

        if (!destinationReached) {

            this.leftTravelled = -toInches(Robot.leftDriveEnc.getQuadraturePosition());
            this.rightTravelled = toInches(Robot.rightDriveEnc.getQuadraturePosition());
            this.distanceTravelled = (this.leftTravelled + this.rightTravelled) / 2.0;

            this.error = INCHES - this.distanceTravelled;
            this.differentialError = this.rightTravelled - this.leftTravelled;

            this.genPower = (this.genPower + (this.error * P_gen_forwardback)) / 2.0;

            if (this.genPower > MAX_POWER) this.genPower = MAX_POWER;
            else if (this.genPower < -MAX_POWER) this.genPower = -MAX_POWER;

            this.diffPower = this.genPower * ((diffPower + (differentialError * P_differential)) / 2.0);

            Robot.r_robotDrive.arcadeDrive(genPower, diffPower);
            // Robot.r_robotDrive.arcadeDrive(Robot.c_controller.getLeftY(), Robot.c_controller.getRightX());

        } else {

            leftTravelled = 0;
            rightTravelled = 0;
            distanceTravelled = 0;

        }

    }

    public void reverse(double INCHES) {

        if (!destinationReached) {

            leftTravelled = toInches(leftEncoder.getQuadraturePosition());
            rightTravelled = -toInches(rightEncoder.getQuadraturePosition());
            distanceTravelled = (leftTravelled + rightTravelled) / 2.0;

            error = INCHES - distanceTravelled;
            differentialError = rightTravelled - leftTravelled;

            genPower = (genPower + (error * P_gen_forwardback)) / 2.0;

            diffPower = genPower * ((rightOffset + (differentialError * P_differential)) / 2.0);

            robotDrive.arcadeDrive(genPower, diffPower);

            destinationReached = distanceTravelled < INCHES;

        } else {

            Robot.mg_leftDrive.set(0);
            Robot.mg_rightDrive.set(0);
            leftTravelled = 0;
            rightTravelled = 0;
            distanceTravelled = 0;

        }

    }

    public void turnRight(double INCHES) {

        if (!destinationReached) {

            leftTravelled = -toInches(leftEncoder.getQuadraturePosition());
            rightTravelled = -toInches(rightEncoder.getQuadraturePosition());
            distanceTravelled = (leftTravelled + rightTravelled) / 2.0;

            error = INCHES - distanceTravelled;

            genPower = (genPower + (error * P_gen_turn)) / 2.0;

            robotDrive.arcadeDrive(0, genPower);

            destinationReached = distanceTravelled < INCHES;

        } else {

            Robot.mg_leftDrive.set(0);
            Robot.mg_rightDrive.set(0);
            leftTravelled = 0;
            rightTravelled = 0;
            distanceTravelled = 0;

        }

    }

    public void turnLeft(double INCHES) {

        if (!destinationReached) {

            leftTravelled = toInches(leftEncoder.getQuadraturePosition());
            rightTravelled = toInches(rightEncoder.getQuadraturePosition());
            distanceTravelled = (leftTravelled + rightTravelled) / 2.0;

            error = INCHES - distanceTravelled;

            genPower = (genPower + (error * P_gen_turn)) / 2.0;

            robotDrive.arcadeDrive(0, genPower);

            destinationReached = distanceTravelled < INCHES;

        } else {

            Robot.mg_leftDrive.set(0);
            Robot.mg_rightDrive.set(0);
            leftTravelled = 0;
            rightTravelled = 0;
            distanceTravelled = 0;

        }

    }
    
}

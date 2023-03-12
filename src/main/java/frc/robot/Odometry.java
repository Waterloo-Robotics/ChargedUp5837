package frc.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Odometry {
    /* Motor Controllers */
    private static MotorControllerGroup rightDrive;
    private static MotorControllerGroup leftDrive;
    private static DifferentialDrive robotDrive;

    /* Drive Encoders */
    public static SensorCollection rightEncoder;
    public static SensorCollection leftEncoder;

    /* Phyiscal Parameters */
    private static double countsPerRev = 4096;
    private static double wheelDiameter = 6.0;
    private static double wheelCircumference = wheelDiameter * Math.PI;
    private static double gearRatio = 1.0;

    /* PID Constants */
    private static double P_general_fwrd = 0.05;
    private static double P_general_turn = 0.03;
    private static double P_differential = 0.01;

    /* Speed Constants */
    private static double MAX_POWER = 0.05;
    private static double MIN_POWER_STRAIGHT = 0;
    private static double MIN_POWER_TURN = 0;

    public double genPower;
    public double diffPower;

    public double distanceTravelled;
    public double error;
    public double differentialError;

    public double rightTravelled;
    public double leftTravelled;

    public double rightTarget;
    public double leftTarget;
    public double forwardTarget;

    public boolean destinationReached;

    public Odometry (MotorControllerGroup leftMotorGroup,
                     MotorControllerGroup rightMotorGroup,
                     DifferentialDrive drivebase,
                     SensorCollection leftEnc,
                     SensorCollection rightEnc)
    {
        Odometry.leftDrive = leftMotorGroup;
        Odometry.rightDrive = rightMotorGroup;
        Odometry.robotDrive = drivebase;
        Odometry.leftEncoder = leftEnc;
        Odometry.rightEncoder =  rightEnc;
    }

    public void update() {

        /* If we haven't reached our target perform the calcualtions, otherwise stop drivebase */
        if (!this.destinationReached) {
            this.rightTravelled = toInches(Odometry.rightEncoder.getQuadraturePosition());
            this.leftTravelled = toInches(-Odometry.leftEncoder.getQuadraturePosition());

            this.distanceTravelled = (this.rightTravelled + this.leftTravelled) / 2.0;
            this.differentialError = this.rightTravelled - this.leftTravelled;

            this.error = this.forwardTarget - this.distanceTravelled;

            this.genPower = clipPowerStraight(this.error * P_general_fwrd);
            this.diffPower = clipPowerStraight(this.differentialError * Odometry.P_differential);
        } else {
            this.genPower = 0;
            this.diffPower = 0;
        }

        /* Check if target reached */
        if (this.distanceTravelled >= this.forwardTarget) {
            this.destinationReached = true;
        }

        Odometry.robotDrive.arcadeDrive(this.genPower, this.diffPower);
    }

    public boolean moveFinished() {
        return this.destinationReached;
    }

    public void forward(double inches) {

        /* Reset static variables */
        resetEncoders();
        this.rightTravelled = 0;
        this.leftTravelled = 0;
        this.distanceTravelled = 0;
        this.error = 0;

        this.genPower = 0;
        this.diffPower = 0;

        this.destinationReached = false;

        this.forwardTarget = inches;
    }

    private static double toInches(double encoderCount) {
        return (encoderCount / Odometry.countsPerRev) * Odometry.gearRatio * Odometry.wheelCircumference;
    }

    public static void resetEncoders() {
        Odometry.rightEncoder.setQuadraturePosition(0, 0);
        Odometry.leftEncoder.setQuadraturePosition(0, 0);
    }

    private static double clipPowerStraight(double power) {
        if (Math.abs(power) < Odometry.MIN_POWER_STRAIGHT) 
        {
            return Odometry.MIN_POWER_STRAIGHT * Math.signum(power);
        } else if (Math.abs(power) > Odometry.MAX_POWER) 
        {
            return Odometry.MAX_POWER * Math.signum(power);
        } else {
            return power;
        }
    }

    





}

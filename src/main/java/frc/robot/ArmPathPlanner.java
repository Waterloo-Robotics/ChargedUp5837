package frc.robot;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPathPlanner {

    /* Safe positions in front and back of frame perimeter */
    ArmPosition homeFront;
    ArmPosition homeBack;

    /* Constructor */
    public ArmPathPlanner(ArmPosition homeFront, ArmPosition homeBack) {

        this.homeFront = homeFront;
        this.homeBack = homeBack;

    }

    /* Plan a path from currentArmPosition to desiredArmPosition */
    public ArmSequence planPath(ArmPosition currentArmPosition, ArmPosition desiredArmPosition) {
        int flag = -1;

        /* Create a new ArmSequence */
        ArmSequence path = new ArmSequence();

        int currentSide = currentArmPosition.returnSide();
        int desiredSide = desiredArmPosition.returnSide();

        SmartDashboard.putNumber("Current Side", currentSide);
        SmartDashboard.putNumber("Current Pos", currentArmPosition.x);
        SmartDashboard.putNumber("Desired Side", desiredSide);
        SmartDashboard.putNumber("Desired Pos", desiredArmPosition.x);

        /* Add current position as first stop */
        path.addPosition(currentArmPosition);

        /* If currently in Front */
        if (currentSide == 1) {
            if (desiredSide == 1) {
                path.addPosition(desiredArmPosition);
                flag = 1;
            }
            else if (desiredSide == 0) {
                path.addPosition(homeFront);
                path.addPosition(desiredArmPosition);
                flag = 2;
            }
            else if (desiredSide == -1) {
                path.addPosition(homeFront);
                path.addPosition(homeBack);
                path.addPosition(desiredArmPosition);
                flag = 3;
            }
            else {
                path.addPosition(currentArmPosition);
                flag = 4;
            }
        } 
        /* If currently within frame perimeter */
        else if (currentSide == 0) {
            if (desiredSide == 1) {
                path.addPosition(homeFront);
                path.addPosition(desiredArmPosition);
                flag = 5;
            }
            else if (desiredSide == 0) {
                path.addPosition(desiredArmPosition);
                flag = 6;
            }
            else if (desiredSide == -1) {
                path.addPosition(homeBack);
                path.addPosition(desiredArmPosition);
                flag = 7;
            } 
            else {
                path.addPosition(currentArmPosition);
                flag = 8;
            }
        } 
        /* If currently in Back */
        else if (currentSide == -1) {
            if (desiredSide == 1) {
                path.addPosition(homeBack);
                path.addPosition(homeFront);
                path.addPosition(desiredArmPosition);
                flag = 9;
            }
            else if (desiredSide == 0) {
                path.addPosition(homeBack);
                path.addPosition(desiredArmPosition);
                flag = 10;
            }
            else if (desiredSide == -1) {
                path.addPosition(desiredArmPosition);
                flag = 11;
            } 
            else {
                path.addPosition(currentArmPosition);
                flag = 12;
            }
        }

        SmartDashboard.putNumber("Flag", flag);

        return path;

    }

}

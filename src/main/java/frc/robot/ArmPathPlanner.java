package frc.robot;

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

        /* Create a new ArmSequence */
        ArmSequence path = new ArmSequence();

        int currentSide = currentArmPosition.returnSide();
        int desiredSide = desiredArmPosition.returnSide();

        SmartDashboard.putNumber("Current Side", currentSide);
        SmartDashboard.putNumber("Current Pos", currentArmPosition.x);
        SmartDashboard.putNumber("Desired Side", desiredSide);
        SmartDashboard.putNumber("Desired Pos", desiredArmPosition.x);

        /* Add current position as first stop */
        path.addPosition(new ArmPosition(currentArmPosition));

        /* If currently in Front */
        if (currentSide == 1) {

            if (desiredSide == 1) {

                path.addPosition(new ArmPosition(desiredArmPosition));

            } else if (desiredSide == 0) {

                path.addPosition(new ArmPosition(homeFront));
                path.addPosition(new ArmPosition(desiredArmPosition));

            } else if (desiredSide == -1) {

                path.addPosition(new ArmPosition(homeFront));
                path.addPosition(new ArmPosition(homeBack));
                path.addPosition(new ArmPosition(desiredArmPosition));

            } else {

                path.addPosition(new ArmPosition(currentArmPosition));

            }

        } 
        /* If currently within frame perimeter */
        else if (currentSide == 0) {
            if (desiredSide == 1) {
                path.addPosition(new ArmPosition(homeFront));
                path.addPosition(new ArmPosition(desiredArmPosition));
            }
            else if (desiredSide == 0) {
                path.addPosition(new ArmPosition(desiredArmPosition));
            }
            else if (desiredSide == -1) {
                path.addPosition(new ArmPosition(homeBack));
                path.addPosition(new ArmPosition(desiredArmPosition));
            } 
            else {
                path.addPosition(new ArmPosition(currentArmPosition));
            }
        } 
        /* If currently in Back */
        else if (currentSide == -1) {
            if (desiredSide == 1) {
                path.addPosition(new ArmPosition(homeBack));
                path.addPosition(new ArmPosition(homeFront));
                path.addPosition(new ArmPosition(desiredArmPosition));
            }
            else if (desiredSide == 0) {
                path.addPosition(new ArmPosition(homeBack));
                path.addPosition(new ArmPosition(desiredArmPosition));
            }
            else if (desiredSide == -1) {
                path.addPosition(new ArmPosition(desiredArmPosition));
            } 
            else {
                path.addPosition(new ArmPosition(currentArmPosition));
            }
        }

        return path;

    }

}

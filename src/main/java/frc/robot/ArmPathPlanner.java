package frc.robot;

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
        
        /* If the current side is not the same as the desired side */
        if (currentArmPosition.returnSide() != desiredArmPosition.returnSide()) {

            /* If currentPosition is in the front */
            if (currentArmPosition.returnSide() == 1) {
                /* If desiredArmPosition is within frame perimeter */
                if (desiredArmPosition.returnSide() == 0) {
                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);
                } 
                /* If desiredArmPosition is in the back */
                else if (desiredArmPosition.returnSide() == -1){
                    path.addPosition(this.homeFront);
                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);
                } 
                /* Something went wrong, don't go anywhere */
                else {
                    path.addPosition(currentArmPosition);
                }
            /* If currentPosition is within frame perimeter */
            } else if (currentArmPosition.returnSide() == 0) {  
                /* If desiredArmPosition is in the front */
                if (desiredArmPosition.returnSide() == 1) {
                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);
                } 
                /* If desiredArmPosition is in the back */
                else if (desiredArmPosition.returnSide() == -1) {
                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);
                } 
                /* Something went wrong, don't go anywhere */
                else {
                    path.addPosition(currentArmPosition);
                }
            /* If currentPosition is in the back */
            } else {
                /* If desiredArmPosition is within frame perimeter */
                if (desiredArmPosition.returnSide() == 0) {
                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);
                }  
                /* If desiredArmPosition is in the front */
                else if (desiredArmPosition.returnSide() == 1){
                    path.addPosition(this.homeBack);
                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);
                } 
                /* Something went wrong, don't go anywhere */
                else {
                    path.addPosition(currentArmPosition);
                }
            }

        } 
        /* desiredArmPosition is on the same side as the currentArmPosition */
        else {

            path.addPosition(desiredArmPosition);

        }

        return path;

    }

}

package frc.robot;

public class ArmPathPlanner {

    ArmPosition homeFront;
    ArmPosition homeBack;

    public ArmPathPlanner(ArmPosition homeFront, ArmPosition homeBack) {

        this.homeFront = homeFront;
        this.homeBack = homeBack;

    }

    public ArmSequence planPath(ArmPosition currentArmPosition, ArmPosition desiredArmPosition) {

        ArmSequence path = new ArmSequence();
        
        if (currentArmPosition.returnSide() != desiredArmPosition.returnSide()) {

            if (currentArmPosition.returnSide() == 1) {

                if (desiredArmPosition.returnSide() == 0) {

                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);

                } else {

                    path.addPosition(this.homeFront);
                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);

                }

            } else if (currentArmPosition.returnSide() == 0) {  

                if (desiredArmPosition.returnSide() == 1) {

                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);

                } else {

                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);

                }

            } else {

                if (desiredArmPosition.returnSide() == 0) {

                    path.addPosition(this.homeBack);
                    path.addPosition(desiredArmPosition);

                } else {

                    path.addPosition(this.homeBack);
                    path.addPosition(this.homeFront);
                    path.addPosition(desiredArmPosition);

                }

            }

        } else {

            path.addPosition(desiredArmPosition);

        }

        return path;

    }

}

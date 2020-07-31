package frc.robot.actions;

import frc.robot.Robot;

public class TurnAction implements Action {

    private double angle;

    private boolean complete = false;

    /*
     *-1: Task failed
     *0: Task not started
     *1: Task in progress
     *2: Task complete
     */
    
    private int status = 0;

    public TurnAction(double target) {
        angle = target;
    }

    public boolean isComplete() {
        return complete;
    }

    public void start() {
        status = 1;

        Robot.base.reset();
        Robot.base.setAngle(angle);
        Robot.base.snapToAngle();

        int counter = 0;

        while (!Robot.base.turnOnTarget()) {
            counter++;
            Robot.base.snapToAngle();

            if (counter == 10000) {
                status = -1;
                return;
            }

            continue;
        }

        complete = true;
        status = 2;
    }

    public int status() {
        return status;
    }
}
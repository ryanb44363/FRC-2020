/*
package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    public Limelight(Pipeline pipeline) {
        int pipelineID;

        switch (pipeline) {
            case RETRO:
                pipelineID = 0;
                break;
            case BALL:
                pipelineID = 1;
                break;
            default:
                throw new IllegalArgumentException();
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID);
    }

    public Pipeline getPipeline() {
        int pipelineID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(-1);

        switch (pipelineID) {
            case 0:
                return Pipeline.RETRO;
            case 1:
                return Pipeline.BALL;
            default:
                return Pipeline.INVALID;
        }
    }

    public void setPipeline(Pipeline pipeline) {
        int pipelineID;

        switch (pipeline) {
            case RETRO:
                pipelineID = 0;
                break;
            case BALL:
                pipelineID = 1;
                break;
            default:
                throw new IllegalArgumentException();
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID);
    }

    public double[] getValues() {
        double[] values = new double[3];

        values[0] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        values[1] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        values[2] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        return values;
    }
}
*/
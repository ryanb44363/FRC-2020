package frc.robot.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSensor {

    private ColorSensorV3 sensor;

    public ColorSensor(I2C.Port port) {
        sensor = new ColorSensorV3(port);
    }

    public int[] getColor() {
        int[] colors = {sensor.getRed(), sensor.getGreen(), sensor.getBlue()};
        return colors;
    }

    public void dashboard() {
        SmartDashboard.putNumber("Red", getColor()[0]);
        SmartDashboard.putNumber("Green", getColor()[1]);
        SmartDashboard.putNumber("Blue", getColor()[2]);
    }
}
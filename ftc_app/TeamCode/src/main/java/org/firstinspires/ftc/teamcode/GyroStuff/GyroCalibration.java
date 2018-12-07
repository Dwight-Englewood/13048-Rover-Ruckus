package org.firstinspires.ftc.teamcode.GyroStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.State;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class GyroCalibration implements Subsystem {
    private BNO055IMU gyro;

    public GyroCalibration(){}

    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        gyro = hwMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void start() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void stop() {

    }

    @Override
    public State getState() {
        return null;
    }

    public float getGyroRotation(AngleUnit unit) {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
    }
}
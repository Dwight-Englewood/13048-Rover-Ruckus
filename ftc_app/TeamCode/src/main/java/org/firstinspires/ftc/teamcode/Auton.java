package org.firstinspires.ftc.teamcode;

/**
 * Created by Kevin on 10/05/18.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.bot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="autonomous",group="Autonomous")
public class Auton {
    bot robot = new bot();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
//      robot.resetServo();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        if(runtime.milliseconds() >= 2000){
            robot.hook.setPower(-0.5);
        }
        else {
            robot.hook.setPower(0);
        }
    }

    @Override
    public void stop() {

    }

}
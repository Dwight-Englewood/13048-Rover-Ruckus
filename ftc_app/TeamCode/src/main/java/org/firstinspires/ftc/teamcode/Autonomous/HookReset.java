package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.bot;

@Autonomous(name="Hook", group="Autonomous")

public class HookReset extends OpMode {
    bot robot = new bot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hook.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        robot.hook.setTargetPosition(7500);
        robot.hook.setPower(1);
        robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void stop(){

    }
}




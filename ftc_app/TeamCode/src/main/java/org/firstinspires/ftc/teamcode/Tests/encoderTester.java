package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.bot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TET",group="Teleop")
public class encoderTester extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    //TensorFlow tensorFlow = new TensorFlow();
    bot robot = new bot();
    boolean wabbo = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
//        robot.resetServo();

        telemetry.addData("Status", "Initialized");
        robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Hook Power", robot.hook.getPower());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        //tensorFlow.init(hardwareMap, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //tensorFlow.start();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y,   gamepad1.left_trigger,gamepad1.right_trigger,wabbo, false);
        //  robot.hook.setTargetPosition(1120 );
      //  robot.hook.setPower(0.5);
       // if(gamepad1.b){robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
      //  else{robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
        if (gamepad1.a) {
            robot.hook.setPower(1);

        } else if (gamepad1.b) {
            robot.hook.setPower(-1);

        } else {
            robot.hook.setPower(0);
        }

        // tensorFlow.getState();
      //  telemetry.addData("Tensor Flow Stats", tensorFlow.getState());
       // telemetry.addData("BL positiong", robot.BL.getCurrentPosition());
       // telemetry.addData("FL positiong", robot.FL.getCurrentPosition());
       // telemetry.addData("BR positiong", robot.BR.getCurrentPosition());
        telemetry.addData("hook positiong", robot.hook.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
//        tensorFlow.stop();
    }
}

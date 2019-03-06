package org.firstinspires.ftc.teamcode.TeleBop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDStuff.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.bot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.BoBot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BoBo Consumes Rice",group="Teleop")

public class TeleBoBo extends OpMode{
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    BoBot robot = new BoBot();
    double kp = 2;
    double kd = 5;
    double ki = 0.0005;
    PIDController pid = new PIDController(kp, ki,kd);
    boolean Command;
    boolean wabbo = false;

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry, false);
//        robot.resetServo();
        telemetry.addData("Status", "Initialized");
        robot.joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hook Power", robot.hook.getPower());
        //   telemetry.addData("Claw Position", robot.claw.getPosition());
    //    robot.joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
        robot.door.setPosition(0.0);
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        double leftPower = Range.clip(gamepad1.left_stick_y, -0.75,0.75);
//        double rightPower = Range.clip(gamepad1.right_stick_y, -0.75, 0.75);
//        robot.tankDriveNoStrafe(gamepad1.left_stick_y, gamepad1.right_stick_y);
        ;

        if(gamepad1.x){wabbo = true; }
        else if (gamepad1.y){wabbo = false;}
        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y,   gamepad1.left_trigger,gamepad1.right_trigger,wabbo, false);

        robot.lift.setPower(gamepad2.left_stick_y);

        if(gamepad1.b){
            robot.joint.setPower(-1.);}
        else if (gamepad1.a){robot.joint.setPower(1.);}
        else{robot.joint.setPower(0);}

        if(gamepad2.a){robot.hook.setPower(1);}
        else if(gamepad2.b){robot.hook.setPower(-1);}
        else{robot.hook.setPower(0);}

      //  robot.lift.setPower(gamepad2.left_stick_y);

        robot.inBOBO.setPower(gamepad2.right_stick_y *0.7);
      //  robot.intake.setPower(gamepad2.right_stick_y);

       // robot.inBOBO.setPower(gamepad2.right_stick_y);
        telemetry.addData("power", robot.FL.getPower());
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}



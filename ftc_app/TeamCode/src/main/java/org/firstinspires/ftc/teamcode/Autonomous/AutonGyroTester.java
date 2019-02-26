package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.bot;

/**
 * Created by Kevin on 10/19/18.
 */


@Autonomous(name = "[OLD]TimeAuton", group = "Testing")

public class AutonGyroTester extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    bot robot = new bot();
    int auto = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        //robot.resetServo();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Initialized");
    //    robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     //   robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (auto) {

            //Time Function Socks
            case 0:
                if (runtime.milliseconds() <= 6250) {
                    robot.hook.setPower(1.0);
                } else if (runtime.milliseconds() > 6250) {
                    robot.hook.setPower(0);
                       auto++;
                } else {
                    robot.hook.setPower(0);
                     auto++;
                }

                break;

            case 1 :
                runtime.reset();
                auto++;
                break;


            case 2:
                if (runtime.milliseconds() <= 1000) {
                    robot.drive(MovementEnum.RIGHTSTRAFE, 0.5);
                }
                else if(runtime.milliseconds() > 1000) {
                    robot.drive(MovementEnum.STOP, 0.0);
                    auto++;
                }
                else {robot.drive(MovementEnum.STOP, 0.0);
                    auto++;
                }
                break;

            case 3:
                runtime.reset();
                auto++;
                break;

            case 4:
                // if (runtime.milliseconds() >= 12000) {
                //enable for color sensor here using DogeCV or OpenCV (Preferably DogeCV)
              /*
                if (runtime.milliseconds() > 7000) {
                    robot.drive(MovementEnum.RIGHTTURN, 0.65);
                }
                auto++;
                */
                if (runtime.milliseconds() <= 3000) {
                    robot.drive(MovementEnum.BACKWARD, 1);
                }
                else if(runtime.milliseconds() > 3000){ robot.drive(MovementEnum.STOP,0);}
                else {robot.drive(MovementEnum.STOP,0);}
                  //   auto++;


                    break;

            case 34:
                if (runtime.milliseconds() <= 10000) {
                    robot.drive(MovementEnum.BACKWARD, 1);
                }
                else if(runtime.milliseconds() > 10000)
               // auto++;
                break;

            case 45:
                if (runtime.milliseconds() > 11000) {
                    robot.claw.setPosition(1);
                }
                auto++;
                break;

            case 5:
                if (runtime.milliseconds() > 13000) {
                    robot.drive(MovementEnum.BACKWARD, 1);
                }
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);

            }
            break;
            //if gold color (RGB value) is detected return value. G
            // Go forward,  go backwards, and turn left.
            // Go forward until distance to wall is 6 inches.
            // Turn 45 degrees, and go forward.
            // Drop the team marker, then back up into the crater.
        }

    }

//        telemetry.addData("degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        telemetry.update();
//        robot.testServos(telemetry);
//        telemetry.update();
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}


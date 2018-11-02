/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.bot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton_Crater", group="Iterative Opmode")
//@Disabled
public class Auton_Crater extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    int auto = 0;
    final int value = 1120;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        //robot.resetServo();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Initialized");

        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        int position = robot.hook.getCurrentPosition();
        telemetry.addData("Encoder Position", position);

        //Figure out tick count √
        //stop and reset encoders √
        //set the target position √
        //set the desired power
        //set to RUN_TO_POSITION
        //wait while isBusy()
        //Stop the motor

        double TICK_COUNT = 1120;
        double fullTurn = 1120;

        //FORWARD
        robot.FL.setTargetPosition(value);
        robot.FR.setTargetPosition(value);
        robot.BL.setTargetPosition(value);
        robot.BR.setTargetPosition(value);

        //FORWARD*2
        robot.FL.setTargetPosition(value*2);
        robot.FR.setTargetPosition(value*2);
        robot.BL.setTargetPosition(value*2);
        robot.BR.setTargetPosition(value*2);

        //BACKWARD
        robot.FL.setTargetPosition(-value);
        robot.FR.setTargetPosition(-value);
        robot.BL.setTargetPosition(-value);
        robot.BR.setTargetPosition(-value);

        //BACKWARD*2
        robot.FL.setTargetPosition(-value*2);
        robot.FR.setTargetPosition(-value*2);
        robot.BL.setTargetPosition(-value*2);
        robot.BR.setTargetPosition(-value*2);

        //RIGHTSTRAFE
        robot.FL.setTargetPosition(-value);
        robot.FR.setTargetPosition(value);
        robot.BL.setTargetPosition(value);
        robot.BR.setTargetPosition(-value);

        //LEFTSTRAFE
        robot.FL.setTargetPosition(value);
        robot.FR.setTargetPosition(-value);
        robot.BL.setTargetPosition(-value);
        robot.BR.setTargetPosition(value);

        //RIGHTTURN
        robot.FL.setTargetPosition(value);
        robot.FR.setTargetPosition(-value);
        robot.BL.setTargetPosition(value);
        robot.BR.setTargetPosition(-value);

        //RIGHTHALFTURN
        robot.FL.setTargetPosition(value/2);
        robot.FR.setTargetPosition(-value/2);
        robot.BL.setTargetPosition(value/2);
        robot.BR.setTargetPosition(-value/2);
        //RIGHTQUARTERTURN
        robot.FL.setTargetPosition(value/4);
        robot.FR.setTargetPosition(-value/4);
        robot.BL.setTargetPosition(value/4);
        robot.BR.setTargetPosition(-value/4);

        //LEFTTURN
        robot.FL.setTargetPosition(-value);
        robot.FR.setTargetPosition(value);
        robot.BL.setTargetPosition(-value);
        robot.BR.setTargetPosition(value);

        //LEFTHALFTURN
        robot.FL.setTargetPosition(-value/2);
        robot.FR.setTargetPosition(value/2);
        robot.BL.setTargetPosition(-value/2);
        robot.BR.setTargetPosition(value/2);

        //LEFTQUARTERTURN
        robot.FL.setTargetPosition(-value/4);
        robot.FR.setTargetPosition(value/4);
        robot.BL.setTargetPosition(-value/4);
        robot.BR.setTargetPosition(value/4);

        //HOOK
        robot.hook.setTargetPosition(value);
        robot.hook.setTargetPosition(-value);

       /*
        while(robot.FL.isBusy()) {
            telemetry.addData("Status", "Running Front Left Full Turn");
            telemetry.update();
        }

        while(robot.FR.isBusy()) {
            telemetry.addData("Status", "Running Front Right Full Turn");
            telemetry.update();
        }

        while(robot.BL.isBusy()) {
            telemetry.addData("Status", "Running Back Left Full Turn");
            telemetry.update();
        }

        while(robot.BR.isBusy()) {
            telemetry.addData("Status", "Running Back Right Full Turn");
            telemetry.update();
        }
        */

        switch (auto) {

            case 0:
                if(robot.hook.getCurrentPosition() >= 1120*3) {
                    robot.hook.setPower(-1);
                    robot.hook.setTargetPosition(value);

                } else {
                    robot.hook.setPower(0);
                }

                auto++;
                break;

            case 1:
                if(robot.FL.getCurrentPosition() >= 1120*3) {
                    robot.FL.setPower(-1);
                    robot.FR.setPower(1);
                    robot.BL.setPower(1);
                    robot.BR.setPower(-1);

                    robot.FL.setTargetPosition(value*3);
                    robot.FR.setTargetPosition(value*3);
                    robot.BL.setTargetPosition(value*3);
                    robot.BR.setTargetPosition(value*3);

                } else {
                    robot.FL.setPower(0);
                    robot.FR.setPower(0);
                    robot.BL.setPower(0);
                    robot.BR.setPower(0);
                }


                auto++;
                break;

            case 2:
                robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                auto++;
                break;

            case 3:
                if(robot.FL.getCurrentPosition() >= 1120*10) {
                    robot.FL.setPower(-1);
                    robot.FR.setPower(-1);
                    robot.BL.setPower(-1);
                    robot.BR.setPower(-1);

                    robot.FL.setTargetPosition(value*10);
                    robot.FR.setTargetPosition(value*10);
                    robot.BL.setTargetPosition(value*10);
                    robot.BR.setTargetPosition(value*10);

                } else {
                    robot.FL.setPower(0);
                    robot.FR.setPower(0);
                    robot.BL.setPower(0);
                    robot.BR.setPower(0);
                }

               // auto++;
                break;

            case 4:
                robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                auto++;
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }
            break;
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


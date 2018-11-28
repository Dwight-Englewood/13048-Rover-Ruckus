package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry; //[Why did you import telemetry twice?]
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop",group="Teleop")
//@Disabled
public class TeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    bot robot = new bot();
    int pos = 0;
    int extPos = 0;
    int currentPosition;
    int targetPosition;
    int command = 0;
    int command2 = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
//      robot.resetServo();
        telemetry.addData("Status", "Initialized");

        telemetry.addData("Hook Power", robot.hook.getPower());
        telemetry.addData("Claw Position", robot.claw.getPosition());

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
        robot.claw.setPosition(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        double leftPower = Range.clip(gamepad1.left_stick_y, -0.75,0.75);
//        double rightPower = Range.clip(gamepad1.right_stick_y, -0.75, 0.75);
//        robot.tankDriveNoStrafe(gamepad1.left_stick_y, gamepad1.right_stick_y);
        //TODO: After competition, comment out tankDriveNoStrafe and enable normal tankDrive for strafable Mechanum Wheels.
        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, false, false);
        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(gamepad1.left_bumper) {
            robot.hook.setPower(0);

        } else if (gamepad1.right_bumper) {
            robot.hook.setPower(1);
        }

            switch (command) {
                case 0:
                    if (!robot.hookLimit.getState()) {
                        command++;
                        break;
                    }

                    if (gamepad1.a) {
                    robot.hook.setPower(1.0);

                } else if (gamepad1.b) {
                    robot.hook.setPower(-1.0);

                } else {
                    robot.hook.setPower(0.0);
                }

                case 1:
                    robot.hook.setPower(-1.0);
                    if (gamepad2.a) {
                    robot.hook.setPower(0);

                    } else if(robot.hookLimit.getState()) {
                        command2 = 0;
                        break;
                    }
                    break;
            }

            switch (command2) {
                case 0:
                    if (!robot.liftLimit.getState()) {
                        command2++;
                        break;
                    }

                    if(gamepad2.right_stick_y > 0.3) {
                        robot.lift.setPower(gamepad2.right_stick_y * -0.75);

                    } else if(gamepad2.right_stick_y < -0.3) {
                        robot.lift.setPower(gamepad2.right_stick_y * -0.75);

                    } else {
                        robot.lift.setPower(0);
                    }

                case 1:
                    robot.lift.setPower(gamepad2.right_stick_y * -0.75);
                    if (gamepad2.right_stick_y < -0.3) {
                        robot.lift.setPower(0);

                    } else if(robot.liftLimit.getState()) {
                        command2 = 0;
                        break;
                    }
                    break;
            }

        if(gamepad2.left_stick_y > 0.3) {
         //   robot.intake.setTargetPosition(2240);
            robot.intake.setPower(gamepad2.left_stick_y * -0.75);
        }

            else if(gamepad2.left_stick_y < -0.3) {
              //  robot.intake.setTargetPosition(2240);
                robot.intake.setPower(gamepad2.left_stick_y * -0.75);

                } else {
                robot.intake.setPower(0);
                }

            if (gamepad2.b) {
                robot.claw.setPosition(0.0);

            } else if (gamepad2.a) {
                robot.claw.setPosition(0.7);
            }

            if (gamepad2.y) {
                robot.dump.setPosition(0.75);

            } else if (gamepad2.x) {
                robot.dump.setPosition(-1);
            }

            telemetry.addData("Lift State", robot.liftLimit.getState());

            telemetry.addData("Hook State", robot.hookLimit.getState());

            telemetry.addData("Command", command);
        telemetry.addData("Auto", command2);

//        telemetry.addData("degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        telemetry.update();
//        robot.testServos(telemetry);
//        telemetry.update();
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}
package org.firstinspires.ftc.teamcode.Tests;
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import org.firstinspires.ftc.teamcode.Hardware.PID;
import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;

import java.util.Locale;

@Autonomous(name = "ENCODER WERPPAR TESTS", group = "Autonomous")
public class EncoderWrapperTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();

    int autonTest = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.claw.setPosition(0.0);
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
        switch (autonTest) {
            case 0:
                try {
                    Thread.sleep(2000);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }
                autonTest++;
                break;

            case 1:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.claw.setPosition(0.0);
                autonTest++;
                break;

            case 2:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 3:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            case 4:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 5:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            case 6:
                robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 7:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            case 8:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 9:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            case 10:
                robot.autonDriveUltimate(MovementEnum.RIGHTTURN, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 11:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            case 12:
                robot.autonDriveUltimate(MovementEnum.LEFTTURN, 750, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    autonTest++;
                }
                break;

            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                autonTest++;
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }
            break;
        }

        telemetry.addData("FL Power", robot.FL.getPower());
        telemetry.addData("FL Cur Pos", robot.FL.getCurrentPosition());
        telemetry.addData("FL Target Pos", robot.FL.getTargetPosition());
        telemetry.addData("Current Case Numbo:", autonTest );
}
        @Override
        public void stop() {
        }
    }
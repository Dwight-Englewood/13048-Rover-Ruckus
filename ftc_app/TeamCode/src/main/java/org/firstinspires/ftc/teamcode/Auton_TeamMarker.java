
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

import android.renderscript.Sampler;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.bot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.GyroCalibration;

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

@Autonomous(name="Auton_TeamMarker", group="Iterative Opmode")
//@Disabled
public class Auton_TeamMarker extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    int auto = 0;
    int turned = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {

        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //hinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            //FL AND FR HAVE REVERSED POWERS

            case 0:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 1:
                robot.hook.setTargetPosition(50000);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.hook.getCurrentPosition() >= 50000 || !robot.hookLimit.getState()){
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 2:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 3:
                robot.autonDrive(MovementEnum.FORWARD, 280 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() >= 280 / 2){
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 4:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 5:
                robot.autonDrive(MovementEnum.RIGHTSTRAFE, 1120 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() >= 1120 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 6:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 7:
                //90 degree left turn
                robot.adjustHeading(-90);
                if(-90 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5){
                    auto++;
                    break;
                }
                break;
/*
                robot.autonDrive(MovementEnum.LEFTTURN, 2364 / 2);
                robot.setPower(1);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.FR.getCurrentPosition() >= 2364 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;
*/
            case 8:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 9:
                robot.autonDrive(MovementEnum.BACKWARD, 2240 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() <= -2240 / 2){
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 10:
                robot.claw.setPosition(0.7);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 11:
                //45 degrees left turn
                robot.adjustHeading(-45);
                if(-45 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5){
                    auto++;
                    break;
                }
                break;
           /*
                robot.autonDrive(MovementEnum.LEFTTURN, 1182 / 2);
                robot.setPower(1);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.FR.getCurrentPosition() >= 1120 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    robot.claw.setPosition(0.0);
                    telemetry.update();
                    auto++;
                }
                break;
*/
            case 12:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 13 :
                robot.autonDrive(MovementEnum.FORWARD, 4480 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() >= 4480 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 14:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 15:
                robot.hook.setTargetPosition(-23000);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.hook.getCurrentPosition() <= -23000){
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 16:
                robot.autonDrive(MovementEnum.STOP, 0);
                robot.setPower(0);
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }

            break;
        }

        telemetry.addData("Hook Current Position", robot.hook.getCurrentPosition() / 1120);
        telemetry.addData("Hook Revo Remaining", ((robot.hook.getTargetPosition() - robot.hook.getCurrentPosition()) / 1120));
        telemetry.addData("Hook Target Position", robot.hook.getTargetPosition() / 1120);

        telemetry.addData("FL Power", robot.FL.getPower());
        telemetry.addData("FR Power", robot.FR.getPower());
        telemetry.addData("BL Power", robot.BL.getPower());
        telemetry.addData("BR Power", robot.BR.getPower());

        telemetry.addData("FL Position", robot.FL.getCurrentPosition());
        telemetry.addData("FL Target Pos", robot.FL.getTargetPosition());

        telemetry.addData("Gyro Degrees ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        telemetry.addData("Auto", auto);
    }

    /*
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     */

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}



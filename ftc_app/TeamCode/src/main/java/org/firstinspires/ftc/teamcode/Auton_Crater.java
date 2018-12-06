
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.bot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.GyroCalibration;

import org.firstinspires.ftc.teamcode.TensorFlow;

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
    TensorFlow tensorFlow = new TensorFlow();

    int auto = 0;
    int turned = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {

        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
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

        //hinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //

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
        tensorFlow.start();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch (auto) {
            //FL AND FR HAVE REVERSED POWERS
            /**Auton_Crater Concept:
             * ***THE PHONE IS ON THE RIGHT SIDE***
             * Turn on tensor flow
             * Drop
             * Backwards
             * Right Strafe
             * 90 Degree Left Turn
             * Leftstrafe slowly
             * Once Detected, go Backwards, then Forwards
             * <------------------------------------------>
             * 90 Degrees Left Turn
             * Forward
             * 135 Degrees Right Turn
             * Backwards
             * Drop Team Marker
             * Forward
             * Reset Hook Position
             */

            case 0:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 1:
                robot.hook.setTargetPosition(50000);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(!robot.hookLimit.getState()){
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 2:
                tensorFlow.getState();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 3:
                robot.autonDrive(MovementEnum.BACKWARD, 280 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() <= -280 / 2){
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
                auto = auto + 3;
                break;


            //leftstrafe


            case 9:
                //45 degrees left turn
                /*
                robot.adjustHeading(-45);
                if(-45 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5){
                    auto++;
                    break;
                }
                break;
           */
                robot.autonDrive(MovementEnum.LEFTTURN, 1100 / 2);
                robot.setPower(0.5);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.FR.getCurrentPosition() >= 1100 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    robot.claw.setPosition(0.0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 10:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 11:
                robot.autonDrive(MovementEnum.LEFTSTRAFE, 1100 / 2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (tensorFlow.getState() == TensorFlow.TFState.CENTER || tensorFlow.getState() == TensorFlow.TFState.LEFT ||tensorFlow.getState() == TensorFlow.TFState.RIGHT ) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                    break;

                } else {
                //    robot.autonTF();
                    auto = 14;
                }
                break;

            case 12:
                if (tensorFlow.getState() == TensorFlow.TFState.LEFT) {
                //    robot.autonTF();
                    auto++;
                    break;

                } else if (tensorFlow.getState() == TensorFlow.TFState.CENTER) {
               //     robot.autonTF();
                    auto++;
                    break;

                } else {
               //     robot.autonTF();
                    auto++;
                    break;
                }

            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 14:
                robot.autonDrive(MovementEnum.LEFTTURN, 1100 / 2);
                robot.setPower(0.5);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.FR.getCurrentPosition() >= 1100 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    robot.claw.setPosition(0.0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 16:
                robot.autonDrive(MovementEnum.FORWARD, 2240 / 2);
                // robot.motorSpeed();
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (robot.BR.getCurrentPosition() >= 2240 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 17:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Furry life is my life- KEvin Chen

            case 18:
                /* robot.adjustHeading(135);
                    if(-135 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5){
*//*
                int gyroVal = (int)robot.fetchHeading();
                robot.gyroCorrect(135, 1, gyroVal, .05, .2);
                if (robot.FL.getPower() == 0) {
                    auto++;
                }
                break;
*/
                /*

                if (robot.adjustHeading(135)){

                        auto++;
                        break;
                    }
                break;
                */

/*
                robot.adjustHeading(135);
                robot.turn(0.2);
                if(135 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5){
                    auto++;
                    break;
                }
                          break;
                          */

                robot.autonDrive(MovementEnum.RIGHTTURN, 1500 / 2);
                robot.motorSpeed();
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (robot.BR.getCurrentPosition() <= -1500 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;


            case 19:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 20:
                robot.autonDrive(MovementEnum.BACKWARD, 4480 / 2);
                //  robot.motorSpeed();
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (robot.BR.getCurrentPosition() <= -4480 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 21:
                robot.claw.setPosition(0.7);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (robot.claw.getPosition() >= 0.7) auto++;
                break;

            case 22:
                robot.autonDrive(MovementEnum.FORWARD, 6720 / 2);
                //    robot.motorSpeed();
                robot.setPower(0.7);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (robot.BR.getCurrentPosition() >= 6720 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 23:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 24:
                robot.hook.setTargetPosition(-24000);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (robot.hook.getCurrentPosition() <= -24000) {
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 25:
                robot.autonDrive(MovementEnum.STOP, 0);
                robot.setPower(0);
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }
            break;
        }
        telemetry.addData("Team Marker Position", robot.claw.getPosition());
        telemetry.addData("Position", tensorFlow.getState());
        telemetry.addData("Case Number:",auto);
        telemetry.update();
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




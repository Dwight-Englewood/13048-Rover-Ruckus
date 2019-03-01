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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.BoBot;
import org.firstinspires.ftc.teamcode.Hardware.PID;
import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;
import java.util.Random;

import java.util.Locale;

@Autonomous(name = "BIG YOTE Fixing Kevin's Code", group = "Autonomous")

public class PIDTester extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    BoBot robot = new BoBot();
    TensorFlow tensorFlow = new TensorFlow();
    int auto = 1;
    TensorFlow.TFState BigThonk, actualState;
    Random rand = new Random();
    int center = 150;
    int left = 600;
    int right = 350;

    int centerBack = 1100;
    int leftBack = 800;
    int rightBack = 1750;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  //      robot.claw.setPosition(0.0);


        /*
    }
        pid.robot.init(hardwareMap, telemetry, false);
        pid.robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        pid.robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        pid.robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        pid.robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
        pid.robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
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
        BigThonk = tensorFlow.getState();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


            switch (auto) {
                case 0:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //              robot.claw.setPosition(0.0);
                    auto++;
                    break;

                case 1:
                    robot.hook.setTargetPosition(23000);
                    robot.hook.setPower(1);
                    robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                    if(!robot.hookLimit.getState() || robot.hook.getCurrentPosition() >= robot.hook.getTargetPosition()){
                        robot.hook.setPower(0);
                        telemetry.update();
                        auto++;
                    }
                    break;

                case 2:
                    if(Math.abs(0- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                        robot.adjustHeading(0);
                    }
                    else if(Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;

                case 3:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tensorFlow.stop();
                    auto++;
                    break;

                case 4:
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 140, 0.5);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 5:
                    BigThonk = tensorFlow.getState();
                    if(BigThonk != TensorFlow.TFState.NOTVISIBLE){auto++;}
                    break;

                case 6:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 7:
                    robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 280, 0.2);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 8:
                    if(Math.abs(-80- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                        robot.adjustHeading(-80);
                    }
                    else if(Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;

                case 9:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 10:
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 280, 0.6);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 11:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                   auto++;
                    break;
                case 12:
                    switch(BigThonk){
                        case RIGHT:
                            robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, center, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                        case LEFT:
                            robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, left, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                        case CENTER:
                            robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, center, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                        case NOTVISIBLE:

                            break;
                }
                    break;
                case 13:
                    tensorFlow.stop();
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 14:
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 275, 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 15:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 16:
                    robot.intake.setPower(1);
                    robot.intake.setTargetPosition(560);
                    robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (Math.abs(robot.intake.getCurrentPosition()) >= Math.abs(robot.intake.getTargetPosition())){
                        robot.intake.setPower(0);
                        auto++;
                    }
                    break;

                case 17:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 18:
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 375, 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 19:
                    if(Math.abs(-170- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                        robot.adjustHeading(-170);
                    }
                    else if(Math.abs(-170 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;
                case 20:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 21:
                    switch(BigThonk){
                        case CENTER:
                            robot.autonDriveUltimate(MovementEnum.BACKWARD, centerBack, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                        case LEFT:
                            robot.autonDriveUltimate(MovementEnum.BACKWARD, leftBack, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                        case RIGHT:
                            robot.autonDriveUltimate(MovementEnum.BACKWARD, rightBack, 0.5);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                auto++;
                            }
                            break;
                    }
                    break;
                case 22:
                    if(Math.abs(-135 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                        robot.adjustHeading(-135);
                    }
                    else if(Math.abs(-135 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;

                case 23:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 24:
                    robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 350, 0.5);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 25:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 26:
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 1000, 0.5);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto+=2;
                    }
                    break;

        /*        case 27:
                    robot.claw.setPosition(0.7);
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    if(robot.claw.getPosition() >= 0.7) {

                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            telemetry.addLine("Sleep Failed");
                            telemetry.update();
                        }
                        auto++;
                    }
                    break;
*/
                case 28:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 29:
                    robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 100, 0.5);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        auto++;
                    }
                    break;

                case 30:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;

                case 31:
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 2200, 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        //auto++;
                    }
                    break;

            }

            // telemetry.addData("FL Power", pid.robot.FL.getPower());
            // telemetry.addData("FL TargetPosition", pid.robot.FL.getTargetPosition());
            // telemetry.addData("FL CurrentPosition", pid.robot.FL.getCurrentPosition());
        //telemetry.addData("Team Marker Position", robot.claw.getPosition());
        //telemetry.addData("Position:", tensorFlow.getState());
       // telemetry.addData("Biggie Thonk:", BigThonk);
        //telemetry.addData("Case Number: ", auto);
       // telemetry.addData("BR POSITION", robot.BR.getCurrentPosition());
        // telemetry.addData("Degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
      //  telemetry.addData("Difference: ", Math.abs(90 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ));
        telemetry.update();

        }
    }


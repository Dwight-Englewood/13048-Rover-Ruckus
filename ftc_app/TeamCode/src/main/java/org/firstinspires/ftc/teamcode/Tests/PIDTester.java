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
import org.firstinspires.ftc.teamcode.Hardware.bot;
import org.firstinspires.ftc.teamcode.Hardware.PID;
import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;
import java.util.Random;

import java.util.Locale;

@Autonomous(name = "BIG YOTE ", group = "Autonomous")

public class PIDTester extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    TensorFlow tensorFlow = new TensorFlow();
    int auto = 1;
    TensorFlow.TFState BigThonk, actualState;
    Random rand = new Random();

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
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.claw.setPosition(0.0);


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
                    // BigThonk = tensorFlow.getState();

                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Sleep Failed");
                        telemetry.update();
                    }

                    auto++;
                    break;

                case 1:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.claw.setPosition(0.0);
                    auto++;
                    break;

                case 2:
                    robot.hook.setTargetPosition(50000);
                    robot.hook.setPower(1);
                    robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                    if (!robot.hookLimit.getState()) {
                        //  BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                        robot.hook.setPower(0);
                        telemetry.update();
                        auto++;
                    }
                    break;

                case 3:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto = 12345;
                    break;
                case 12345:
                    if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                        robot.adjustHeading(0);
                    } else if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                        // robot.tankDrive(0, 0, 0, 0, false, false);
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;
                case 12346:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto = 4;
                    break;
                case 4:
                    robot.autonDrive(MovementEnum.BACKWARD, 280 / 2);
                    robot.setPower(0.5);
                    robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (robot.BR.getCurrentPosition() <= -280 / 2) {
                        robot.drive(MovementEnum.STOP, 0);
                        telemetry.update();
                        auto++;
                    }
                    break;

                case 5:
                    BigThonk = tensorFlow.getState();
                    if (BigThonk != TensorFlow.TFState.NOTVISIBLE) {
                        auto++;
                   }
                    break;
                case 6:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;


                case 7:
                    robot.autonDrive(MovementEnum.RIGHTSTRAFE, 1120 / 4);
                    robot.setPower(0.2);
                    robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (robot.BR.getCurrentPosition() >= 1120 / 4) {
                        robot.drive(MovementEnum.STOP, 0);
                        telemetry.update();
                        auto++;
                    }
                    break;
                case 8:
                    if (Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                        robot.adjustHeading(-80);
                    } else if (Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                        // robot.tankDrive(0, 0, 0, 0, false, false);
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;
                case 9:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 10:
                    robot.autonDrive(MovementEnum.FORWARD, 1120 / 4);
                    robot.setPower(0.6);
                    robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.BR.getCurrentPosition() >= 1120 / 4) {
                        robot.drive(MovementEnum.STOP, 0);
                        telemetry.update();
                        auto++;
                    }
                    break;
                case 11:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                   auto++;
                    break;
                case 12:
                  switch(BigThonk){
                      case LEFT:
                          robot.autonDrive(MovementEnum.LEFTSTRAFE, 600);
                          robot.setPower(0.2);
                          robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                          if(Math.abs(robot.BR.getCurrentPosition()) >= 600) {
                              robot.drive(MovementEnum.STOP, 0);
                              telemetry.update();
                              auto++;
                              // auto++;
                             // auto = 3444;
                          }
                          break;
                      case RIGHT:
                          robot.autonDrive(MovementEnum.RIGHTSTRAFE, 600);
                          robot.setPower(0.2);
                          robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                          if(Math.abs(robot.BR.getCurrentPosition()) >= 600) {
                              robot.drive(MovementEnum.STOP, 0);
                              telemetry.update();
                              auto++;
                              // auto++;
                             // auto = 3444;
                          }
                          break;
                      case CENTER:
                          robot.autonDrive(MovementEnum.LEFTSTRAFE, 1120 / 6);
                          robot.setPower(0.5);
                          robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                          if(Math.abs(robot.BR.getCurrentPosition()) >= 1120 / 6) {
                              robot.drive(MovementEnum.STOP, 0);
                            //  auto = 15;
                              auto++;
                              telemetry.update();

                          }
                          break;
                      case NOTVISIBLE:
                          //int yeet = rand.nextInt(3)+1;
                         // if (yeet == 1 ){BigThonk = TensorFlow.TFState.CENTER;}
                         // else if (yeet ==2){BigThonk = TensorFlow.TFState.LEFT;}
                         // else if(yeet ==3){BigThonk = TensorFlow.TFState.RIGHT;}
                          BigThonk = TensorFlow.TFState.LEFT;
                          break;
                          }
                          break;
                case 13:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 14:
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 840 , 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        telemetry.update();
                        auto++;
                    }
                    break;
                case 15:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 16:
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 840 , 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        telemetry.update();
                        auto++;
                    }
                    break;
                case 17:
                    if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                        robot.adjustHeading(0);
                    } else if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                        // robot.tankDrive(0, 0, 0, 0, false, false);
                        robot.drive(MovementEnum.STOP, 0);
                        auto++;
                    }
                    break;
                case 18:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 19:
                    switch (BigThonk){
                        case LEFT:
                            robot.autonDriveUltimate(MovementEnum.FORWARD, (int) (560 * 1.5), 0.4);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                telemetry.update();
                                auto++;
                            }
                            break;
                        case RIGHT:
                            robot.autonDriveUltimate(MovementEnum.FORWARD, 560 * 5, 0.4);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                telemetry.update();
                                auto++;
                            }
                            break;
                        case CENTER:
                            robot.autonDriveUltimate(MovementEnum.FORWARD, 560 * 3, 0.4);
                            if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                                telemetry.update();
                                auto++;
                            }
                            break;
                        case NOTVISIBLE:
                            BigThonk = TensorFlow.TFState.LEFT;
                            break;
                    }
                    break;
                case 20:
                    if(Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                        robot.adjustHeading(45);
                    }
                    else if(Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3){
                        // robot.tankDrive(0, 0, 0, 0, false, false);
                        robot.drive(MovementEnum.STOP,0);
                       // auto = 313;
                        auto++;
                    }
                    break;
                case 21:
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto++;
                    break;
                case 22:
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 1400, 0.4);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        telemetry.update();
                        auto++;
                    }
                    break;
                case 23:
                    robot.claw.setPosition(0.7);
                    robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    if(robot.claw.getPosition() >= 0.7)

                        try {
                            Thread.sleep(2000);
                        }
                        catch (InterruptedException e){
                            telemetry.addLine("Sleep Failed");
                            telemetry.update();
                        }
                    auto++;
                    break;

                case 24:
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 560 * 4, 0.7);
                    if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                        telemetry.update();
                      //  auto++;
                    }
                    break;




            /*
            case -1:
                pid.robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 0:
                pid.robot.pidTest(MovementEnum.FORWARD, 1000);

                if (Math.abs(pid.robot.FL.getCurrentPosition()) >= Math.abs(pid.robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 1:
                pid.robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
                */
            }

            // telemetry.addData("FL Power", pid.robot.FL.getPower());
            // telemetry.addData("FL TargetPosition", pid.robot.FL.getTargetPosition());
            // telemetry.addData("FL CurrentPosition", pid.robot.FL.getCurrentPosition());
        telemetry.addData("Team Marker Position", robot.claw.getPosition());
        telemetry.addData("Position", tensorFlow.getState());
        telemetry.addData("BiggieThonk", BigThonk);
        telemetry.addData("Case Number: ", auto);
        telemetry.addData("BR POSITION", robot.BR.getCurrentPosition());
        telemetry.addData("Degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Difference: ", Math.abs(90 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ));
        telemetry.update();

        }
    }


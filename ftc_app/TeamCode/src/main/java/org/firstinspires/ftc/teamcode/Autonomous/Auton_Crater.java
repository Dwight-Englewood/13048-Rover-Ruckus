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

        package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.PID;
import org.firstinspires.ftc.teamcode.Hardware.bot;

import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;

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

@Autonomous(name="[OLD]AutonCrater", group="Autonomous")

public class Auton_Crater extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    PID pid = new PID();
    TensorFlow tensorFlow = new TensorFlow();

    TensorFlow.TFState BigThonk, actualState;

    int auto = 0;

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
    /*    if (runtime.milliseconds() >= 29000) {
            robot.autonDrive(MovementEnum.STOP, 0);
            robot.hook.setPower(0);
        }
*/
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
                try {
                    Thread.sleep(2000);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }

                auto++;
                break;

            case 1:
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 2:
                robot.hook.setTargetPosition(50000);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BigThonk =  tensorFlow.getState();

                if(!robot.hookLimit.getState()){
                    BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 3:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 140, 0.2);
                auto++;
                break;

            case 4:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 550, 0.2);
                auto++;
                break;

            case 5:
                if(Math.abs(90 - robot.fetchHeading()) > 3) {
                    robot.adjustHeading(90);

                } else if(Math.abs(90 - robot.fetchHeading()) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

            case 6:
                robot.autonDrive(MovementEnum.LEFTSTRAFE, 550);
                robot.setPower(0.2);
                actualState = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (actualState == TensorFlow.TFState.CENTER || actualState == TensorFlow.TFState.LEFT ||actualState == TensorFlow.TFState.RIGHT ) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.addLine("Visible");
                    telemetry.update();
                    auto++;
                    break;

                } else if(actualState == TensorFlow.TFState.NOTVISIBLE){
                    telemetry.addLine("Not Visible");
                    telemetry.update();
                    auto = 9;
                }
                break;

            case 7:
                if (tensorFlow.getState() == TensorFlow.TFState.CENTER || tensorFlow.getState() == TensorFlow.TFState.LEFT || tensorFlow.getState() == TensorFlow.TFState.RIGHT) {
                    robot.autonDriveUltimate(MovementEnum.BACKWARD, 280, 0.5);
                }
                auto++;
                break;

            case 8:
                if (tensorFlow.getState() == TensorFlow.TFState.CENTER || tensorFlow.getState() == TensorFlow.TFState.LEFT || tensorFlow.getState() == TensorFlow.TFState.RIGHT) {
                    robot.autonDriveUltimate(MovementEnum.FORWARD, 280, 0.5);
                }
                auto++;
                break;

            case 9:
                if(Math.abs(90 - robot.fetchHeading()) > 3) {
                    robot.adjustHeading(90);

                } else if(Math.abs(90 - robot.fetchHeading()) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

            case 10:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 1120, 0.2);
                auto++;
                break;

            case 11:
                if(Math.abs(-135 - robot.fetchHeading()) > 3) {
                    robot.adjustHeading(-135);

                } else if(Math.abs(-135 - robot.fetchHeading()) < 3) {
                    robot.drive(MovementEnum.STOP,0);
                    auto++;
                }
                break;

            case 12:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 2240, 0.2);
                auto++;
                break;

            case 13:
                robot.claw.setPosition(0.7);

                try {
                    Thread.sleep(2000);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }

                auto++;
                break;

            case 14:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 3360, 0.7);
                auto++;
                break;

            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.autonDriveUltimate(MovementEnum.STOP, 0, 0);
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }
            break;
        }

        telemetry.addData("Team Marker Position", robot.claw.getPosition());

        telemetry.addData("Position", tensorFlow.getState());
        // telemetry.addData("BiggieThonk", BigThonk);

        telemetry.addData("Degrees: ", robot.fetchHeading());

        telemetry.addData("Case Number:",auto);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }
}



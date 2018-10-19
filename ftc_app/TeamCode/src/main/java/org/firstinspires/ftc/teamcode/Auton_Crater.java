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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.bot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.AutonBase.*;

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
    bot robot = new bot();
    AutonBase auto = ZERO;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        //robot.resetServo();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Initialized");
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

            case ZERO:
                if (runtime.milliseconds() <= 5000) {
                    robot.hook.setPower(0.5);
                } else if (runtime.milliseconds() > 5000) {
                    robot.hook.setPower(0);
                } else {
                    robot.hook.setPower(0);
                }
                auto = ONE;
                break;

            case ONE:
                if (runtime.milliseconds() > 6000) {
                    robot.drive(MovementEnum.BACKWARD, 0.25);
                }
                auto = TWO;
                break;

            case TWO:
                // if (runtime.milliseconds() >= 12000) {
                //enable for color sensor here using DogeCV or OpenCV (Preferably DogeCV)
                if (runtime.milliseconds() > 7000) {
                    robot.drive(MovementEnum.RIGHTTURN, 0.65);
                }
                auto = THREE;
                break;

            case THREE:
                if (runtime.milliseconds() > 8000) {
                    robot.drive(MovementEnum.FORWARD, 1);
                }
                auto = FOUR;
                break;

            //    } else if (runtime.milliseconds() > 1600) {
            //       robot.gyroTurn(0.5, -45.0);

            case FOUR:
                if (runtime.milliseconds() > 10000) {
                    robot.claw.setPosition(1);
                }
                auto = FIVE;
                break;

            case FIVE:
                if (runtime.milliseconds() > 12000) {
                    robot.drive(MovementEnum.BACKWARD, 1);
                }
                break;

            default: {
                robot.drive(MovementEnum.STOP, 0);
            }
            break;

            /**case SIX:
             if (runtime.milliseconds() > 22000) {
             robot.drive(MovementEnum.LEFTTURN, 0.5);
             }
             break;

             case SEVEN:
             if (runtime.milliseconds() > 22750) {
             robot.drive(MovementEnum.FORWARD, 0.25);
             }
             break;

             case EIGHT:
             if (runtime.milliseconds() > 24750) {
             robot.drive(MovementEnum.LEFTTURN, 0.5);
             }
             break;

             case NINE:
             if (runtime.milliseconds() > 26000) {
             robot.drive(MovementEnum.FORWARD, 1);
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
             **/
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

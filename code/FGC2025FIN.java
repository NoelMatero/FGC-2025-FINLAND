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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.FGC2025FINPIDCONTROLLER;

enum ClimberState {
    IDLE,
    MANUAL_UP,
    MANUAL_DOWN,
    HOLD_POSITION,
    CALIBRATING,
    AUTOMATIC_CLIMB
}


@TeleOp(name="2025-FGC-FIN", group="Linear Opmode")
public class FGC2025FIN extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private static final int TICKS_PER_REV = 560;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    private FGC2025FINPIDCONTROLLER climberPID = new FGC2025FINPIDCONTROLLER(0.0025, 0.000002, 0.001);

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collect_balls1 = null;
    private DcMotor collect_balls2 = null;
    private DcMotor climber = null;
    private DcMotor accelerator = null;
    private Servo servo1 = null;
    private Servo servo2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        leftDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "left_drive");
        collect_balls1 = hardwareMap.get(DcMotor.class, "intake");
        collect_balls2 = hardwareMap.get(DcMotor.class, "intake2");
        climber = hardwareMap.get(DcMotor.class, "climber");
        accelerator = hardwareMap.get(DcMotor.class, "accelerator");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        double last_angle = -1.0;
        boolean climbed = false;
        boolean holdPosition = false;
        boolean calibrate = false;
        boolean automatic_climb = false;
        double target = 0.0;

        double curLeftPower = 0.0;
        double curRightPower = 0.0;

        double softStart = 0.025;

        double lastClimberAngle = 0.0;
        double lastClimberTime = 0.0;
        double Timer2 = 0.0;
        boolean Stalled = false;

        double STALL_ANGLE_CHANGE = 2.0;
        double STALL_TIME_CHANGE = 0.5;

        ClimberState climberState = ClimberState.IDLE;

        while (opModeIsActive()) {
            double leftPower;
            double rightPower;
            double collect_balls1Power;
            double climber_power = 0.0;
            double accelerator_power = 0.0;

            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            curLeftPower += Range.clip(leftPower - curLeftPower, -softStart , softStart);
            curRightPower += Range.clip(rightPower - curRightPower, -softStart, softStart);

            leftDrive.setPower(curLeftPower);
            rightDrive.setPower(curRightPower);

            if (gamepad2.dpad_right) {accelerator_power = 1.0;}
            collect_balls1Power = Range.clip(gamepad2.right_trigger-gamepad2.left_trigger, -1.0, 1.0);
            if (gamepad2.right_bumper) {collect_balls1Power = 0.3;}
            else if (gamepad2.left_bumper) {collect_balls1Power = -0.3;}

            int currentPosition = climber.getCurrentPosition();
            double angle = currentPosition * DEGREES_PER_TICK;

            if (gamepad2.y) {
                climberState = ClimberState.MANUAL_UP;
            }  else if (gamepad2.a) {
               climberState = ClimberState.MANUAL_DOWN;
            } else if (gamepad2.dpad_up) {
               climberState = ClimberState.AUTOMATIC_CLIMB;
            } else if (gamepad2.dpad_down) {
                climberState = ClimberState.IDLE;
            } else if (gamepad2.x) {
                target = 180.0;
                climberPID.reset();
                climberState = ClimberState.CALIBRATING;
            } else if (!gamepad2.y && climbed) {
                last_angle = angle;
                climberPID.reset();
                climberState = ClimberState.HOLD_POSITION;
            }

            switch (ClimberState) {
                case (ClimberState.IDLE) {
                    climber_power = 0.0;
                }

                case (ClimberState.MANUAL_UP) {
                    climber_power = 1.0;
                }

                case (ClimberState.MANUAL_DOWN) {
                    climber_power = -1.0;
                }

                case (ClimberState.HOLD_POSITION) {
                    double pidOutput = climberPID.calculate(last_angle, angle);
                    climber_power = Range.clip(pidOutput, -1.0, 1.0);
                    break;
                }

                case (ClimberState.CALIBRATING) {
                    double pidOutput = climberPID.calculate(target, angle);
                    climber_power = Range.clip(pidOutput, -1.0, 1.0);
                    break;
                }

                case (ClimberState.AUTOMATIC_CLIMB) {
                    double currentTime = runtime.seconds();
                    double deltaTime = currentTime - lastClimberTime;
                    double deltaAngle = Math.abs(angle - lastClimberAngle);

                    lastClimberTime = currentTime;
                    lastClimberAngle = angle;

                    if (deltaAngle < STALL_ANGLE_CHANGE) {
                        Timer2 += deltaTime;
                    } else {
                        Timer2 = 0.0;
                    }

                    if (Timer2 > STALL_TIME_CHANGE) {
                        Stalled = true;
                        automatic_climb = false;
                        holdPosition = true;
                        last_angle = angle;
                        climberPID.reset();
                    } else {
                        climber_power = 1.0;
                    }

                    break;
                }
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            collect_balls1.setPower(collect_balls1Power);
            collect_balls2.setPower(-collect_balls1Power);
            climber.setPower(climber_power);
            accelerator.setPower(accelerator_power);
        }
    }
}

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
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu

 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="2025-FGC-FIN", group="Linear Opmode")

public class FGC2025FIN extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    
    private static final int TICKS_PER_REV = 560;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;
    
    private FGC2025FINPIDCONTROLLER climberPID = new FGC2025FINPIDCONTROLLER(0.0025, 0.000002, 0.001);

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collect_balls1 = null;
    private DcMotor climber = null;
    private DcMotor accelerator = null;
    private Servo servo1 = null;
    private Servo servo2 = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "left_drive");
        collect_balls1 = hardwareMap.get(DcMotor.class, "intake");
        climber = hardwareMap.get(DcMotor.class, "climber");
        accelerator = hardwareMap.get(DcMotor.class, "accelerator");

       
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
 
        // Wait for the game to start (driver presses PLAY)
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

        // pienempi nii hitaampi
        double wheelNopeutus = 0.0085;
        
        double lastClimberAngle = 0.0;
        double lastClimberTime = 0.0;
        double Timer2 = 0.0;
        boolean Stalled = false;

        double STALL_ANGLE_THRESHOLD = 2.0;   
        double STALL_TIME_THRESHOLD = 0.5;    
        
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double collect_balls1Power;
            double climber_power = 0.0;
            double accelerator_power = 0.0;
        
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
        
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            
            curLeftPower += Range.clip(leftPower - curLeftPower, -wheelNopeutus, wheelNopeutus);
            curRightPower += Range.clip(rightPower - curRightPower, -wheelNopeutus, wheelNopeutus);

            leftDrive.setPower(curLeftPower);
            rightDrive.setPower(curRightPower);

            if (gamepad2.dpad_right) {accelerator_power = 0.5;}
            collect_balls1Power = Range.clip(gamepad2.right_trigger-gamepad2.left_trigger, -1.0, 1.0);
            if (gamepad2.right_bumper) {collect_balls1Power = 0.3;}
            else if (gamepad2.left_bumper) {collect_balls1Power = -0.3;}
            
            int currentPosition = climber.getCurrentPosition();
            double angle = currentPosition * DEGREES_PER_TICK;
            
            if (gamepad2.y) {
                climber_power = 1;
                climbed = true;
                holdPosition = false;
            }  else if (gamepad2.a) {
                climber_power = -1;
                holdPosition = false;
                climbed = false;
            } else if (gamepad2.dpad_up) {
                automatic_climb = true;
                holdPosition = false;
                calibrate = false;
                climbed = false;
            } else if (gamepad2.x) {
                target = 180.0; 
                holdPosition = false;
                calibrate = true;
                climbed = false;
                climberPID.reset();
            } else if (!gamepad2.y && climbed) {
                last_angle = angle;
                climbed = false;
                holdPosition = true;
                calibrate = false;
                climberPID.reset();
            }else if (holdPosition) {
                double pidOutput = climberPID.calculate(last_angle, angle);
                climber_power = Range.clip(pidOutput, -1.0, 1.0);
            } else if (calibrate) {
                double pidOutput = climberPID.calculate(target, angle);
                climber_power = Range.clip(pidOutput, -1.0, 1.0);

    
                if (Math.abs(target - angle) < 100.0) { 
                    calibrate = false;
                    holdPosition = true;
                    last_angle = target;
                    climberPID.reset();
                }
            } /*else if (automatic_climb) {
                double currentTime = runtime.seconds();
                double deltaTime = currentTime - lastClimberTime;
                double deltaAngle = Math.abs(angle - lastClimberAngle);

                lastClimberTime = currentTime;
                lastClimberAngle = angle;

                if (deltaAngle < SMALLEST_ANGLE_CHANGE) {
                    stallTimer += deltaTime;
                } else {
                    stallTimer = 0.0; 
                }

                if (stallTimer > STALL_TIME_THRESHOLD) {
                    isStalled = true;
                    automatic_climb = false;
                    holdPosition = true;
                    last_angle = angle; 
                    climberPID.reset();
                    telemetry.addLine("vaithuu");
                } else {
                    climber_power = 1.0;
                }
            }*/
     
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            collect_balls1.setPower(collect_balls1Power);
            climber.setPower(climber_power);

            // Show the elapsed game time and wheel power.
            double pidOutput = climberPID.calculate(target, angle);
            double debug = Range.clip(pidOutput, -1.0, 1.0);
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motor Angle", "%.2f deg", angle);
            telemetry.addData("Lastr Angle", "(%.2f)", last_angle);
            telemetry.addData("Climber power", "(%.2f)", climber_power);
            telemetry.addData("Intake poewr", "(%.2f)", collect_balls1Power);
            telemetry.addData("Angle difference: ", "(%.2f)", Math.abs(last_angle-angle));
            telemetry.addData("Collectors", "(%.2f)", collect_balls1Power);
            telemetry.addData("debug", "(%.2f)", debug);
            telemetry.update();
        }
    }
}

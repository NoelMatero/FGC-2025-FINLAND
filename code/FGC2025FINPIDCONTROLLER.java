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

public class FGC2025FINPIDCONTROLLER {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;

    // kP = proportionaalikerroin — eli reagoi virheen suuruutee
    // kI = integraalikerroin - eli katsoo virheen kertymisen ajassa
    // kD = derivaattakerroin - eli reagoi virheen muutoksen nopeuteen
    public FGC2025FINPIDCONTROLLER(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Control_loop_example
    // tost linkist näkee tän matikan
    public double calculate(double target, double current) {
        // laske virhe
        double error = target - current;
        // jos alle tämä niin antaa olla
        double minError = 150.0;

        if (Math.abs(error) < minError) {
            return 0;
        }
        double currentTime = System.nanoTime() / 1e9;
        // dt - ajan erotus
        double dt = currentTime - lastTimestamp;
        if (lastTimestamp == 0) dt = 0;

        // kertoo kuinka paljon virhettä on kertynyt ajassa
        integralSum += error * dt;
        // derivaatta virheen muutoksen nopeudesta
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        // tää on se taika siihen miten moottorin voima lasketaan
        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;
        lastTimestamp = currentTime;

        if (Math.abs(error) <= 500) {
            output /= 5.0;
        }

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTimestamp = 0;
    }
}


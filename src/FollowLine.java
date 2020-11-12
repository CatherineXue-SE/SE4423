import java.text.DecimalFormat;

import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class FollowLine {

	EV3 ev3;
	TextLCD lcd;
	RegulatedMotor rightWheel;
	RegulatedMotor leftWheel;
	
	public static void main(String[] args) {
		
		new FollowLine();
		
	}
	
	public FollowLine() {
		
		ev3 = (EV3) BrickFinder.getLocal();
		lcd = ev3.getTextLCD();
		
		rightWheel = Motor.B;
		leftWheel = Motor.C;
		
		Sound.beepSequenceUp();
		Button.waitForAnyPress();
		
		followLine();
		
		Sound.beepSequence();
	}
	
	private void followLine() {
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
		
		SensorMode rgbMode = colorSensor.getRGBMode();
		float[] rgbSample = new float[rgbMode.sampleSize()];
		float red = 0.0f;
		float green = 0.0f;
		float blue = 0.0f;
		
		float lowerBounds = 0.03f; //observed lowest acceptable value
		float upperBounds = 0.09f; //observed highest acceptable value
		float mid = 0.0f;
		int turns =0;
		boolean goingForward =true;
		int leftSpeed = 0;
		int rightSpeed = 0;
		int maxSpeed = 700/2;
		int normSpeed = 600/2;
		leftWheel.setSpeed(400);
		rightWheel.setSpeed(400);
		
		leftWheel.forward();
		rightWheel.forward();
		
		DecimalFormat decFormat = new DecimalFormat("#.###");
		
		while(true) {
			rgbMode.fetchSample(rgbSample, 0);
			red = rgbSample[0];//check this if you have a logical error it should be red
		
			green = rgbSample[1];//green
			blue = rgbSample[2];
			
			lcd.clear();
			lcd.drawString("Green: " + decFormat.format(green), 1, 1);
			lcd.drawString("Red: " + decFormat.format(red), 1, 1);
			
			mid = (lowerBounds + upperBounds) / 2;

			if (green > 0.060f && red < 0.060f && blue < 0.060f) {//stop on green lines
				break;
			}

			
			if(red >0.190f && blue < 0.080f ) {
				goingForward = true;
				turns = 0;
				leftSpeed= maxSpeed;
				rightSpeed = maxSpeed;
			}else{
				turns++;
				if(turns <= 50) {
					goingForward = false;
					leftSpeed = normSpeed;
					rightSpeed = 0;
				}

				else if(turns > 50 && turns <= 100){
					goingForward = true;
					
					}

				else if(turns > 100 && turns <= 150){
					goingForward = false;
					rightSpeed = normSpeed;
					leftSpeed = 0;				
					}
				
				else if(turns > 150 && turns <= 200){
					goingForward = true;		
					}		
				
				//hard turns
				else if(turns > 200 && turns <= 300){
					goingForward = false;
					leftSpeed = normSpeed;
					rightSpeed = 0;				
					}
				
				else if(turns > 300 && turns <= 400){
					goingForward = true;		
					}		
				
				
				else if(turns > 400 && turns <= 500){
					goingForward = false;
					rightSpeed = normSpeed;
					leftSpeed = 0;				
					}
				
				else if(turns > 500 && turns <= 600){
					goingForward = true;		
					}		
				
				
				
				
				
				if (turns == 600) {
					turns =0;
				}
		}
			
						
			
			// Update speeds
			leftWheel.setSpeed(leftSpeed);
			rightWheel.setSpeed(rightSpeed);
			
			// Sometimes after setSpeed(0) motors would stop completely
			
			if(goingForward) {
				leftWheel.forward();
				rightWheel.forward();
			}
			else {
				leftWheel.backward();
				rightWheel.backward();
					
			}
			// Output data for debugging
			lcd.drawString(String.valueOf(leftWheel.isStalled()), 1, 2);
			lcd.drawString(String.valueOf(rightWheel.isStalled()), 1, 3);
			lcd.drawString(String.valueOf(leftSpeed), 1, 4);
			lcd.drawString(String.valueOf(rightSpeed), 1, 5);
		}
		
		rightWheel.stop();
		leftWheel.stop();
		
		colorSensor.close();
		rightWheel.stop();
		leftWheel.stop();
	}
	
	private void moveAndMeasure() {

		
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		SampleProvider sampleProvider = irSensor.getDistanceMode();
		
		int sampleRate = 100;
		int travelTime = 10000;
		int iterations = travelTime / sampleRate;
		
		//rightWheel.setPower(50);
		//leftWheel.setPower(50);
		
		for(int i = 0; i < iterations; i++) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			
			float distance = sample[0];
			System.out.println(i + ": " + distance);
			
			Delay.msDelay(sampleRate);
		}
		
		rightWheel.stop();
		leftWheel.stop();
		
		rightWheel.close();
		leftWheel.close();
		irSensor.close();
	}
}

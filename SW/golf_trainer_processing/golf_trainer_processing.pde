/* 
* GolfTrainer Processing sketch
*
* Author: Samuele Mazzei
* Date: 11/09/2019
*
*/

import controlP5.*; 
import oscP5.*;
import netP5.*;
import java.util.Arrays;
  
OscP5 oscP5;
NetAddress myRemoteLocation;

ControlP5 cp5;

Knob launch_angle;
Slider bar_roll;
Slider2D end_pitch;
Slider swing_rotation;
Slider launch_power;
Textlabel text_box;

Group row_1;
int row_2 = 450;

ArrayList<String> valueList;

final int WINDOW_WIDTH = 1500;
final int WINDOW_HEIGHT = 1000;

int BACKGROUND_COLOR = #FFFCF0;
int FOREGROUND_COLOR = #100F0F;
int UI_COLOR = #E6E4D9;
int GREEN = #879A39;
int RED = #D14D41;
int BLUE = #4385BE;

int MIN_SWING_HEIGHT = 0;
int MAX_SWING_HEIGHT = 100;
int MAX_LAUNCH_ANGLE = 50;
int MIN_LAUNCH_ANGLE = -50;

void setup() {
  //size(1500,1000);
  fullScreen();
  
  oscP5 = new OscP5(this,32000); /* start oscP5, listening for incoming messages at port 12000 */
  myRemoteLocation = new NetAddress("127.0.0.1",12000);
  
  
  cp5 = new ControlP5(this);
  cp5.setFont(createFont("Arial",16));

  row_1 = cp5.addGroup("Row 1").setPosition(WINDOW_WIDTH/2 - 250, 100).hideBar();
   
    end_pitch = cp5.addSlider2D("Player Position")
          .setSize(400,200)
          .setGroup(row_1)
          .setMaxX(100)
          .setMinX(0)
          .setMaxY(10)
          .setMinY(0)
          .setColorValueLabel(FOREGROUND_COLOR)
          .setColorCaptionLabel(FOREGROUND_COLOR)
               .setColorBackground(UI_COLOR)

          ;
     
    swing_rotation = cp5.addSlider("Swing Rotation")
      .setPosition(450,0)
     .setSize(50,200)
     .setRange(MIN_SWING_HEIGHT, MAX_SWING_HEIGHT)
     .setValue(0)
     .setColorBackground(UI_COLOR)
     .setGroup(row_1)
      .setColorValueLabel(FOREGROUND_COLOR)
      .setColorCaptionLabel(FOREGROUND_COLOR)
     ;    

     text_box = cp5.addTextlabel("Player Position Label")
      .setText("A little bit on the left")
      .setFont(createFont("Arial", 20))
      .setPosition(WINDOW_WIDTH/2 - 80, 370)
      .setColorValueLabel(FOREGROUND_COLOR)
      .setColorCaptionLabel(FOREGROUND_COLOR)
      .setColorBackground(UI_COLOR);
     
    launch_angle = cp5.addKnob("Launch Angle")  // Define the knob function name
    .setPosition(WINDOW_WIDTH/4, row_2)  // Knob position
    .setSize(200, 200)  // Knob size
    .setRange(MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE)  // Set knob range to -50 to 50 degrees
    .setValue(0)  // Initial value at the center (0 degrees)
    .setNumberOfTickMarks(10)  // Set the number of tick marks
    .setColorForeground(BLUE)  // Set the knob color
          .setColorValueLabel(FOREGROUND_COLOR)
      .setColorCaptionLabel(FOREGROUND_COLOR)
           .setColorBackground(UI_COLOR);


    launch_power = cp5.addSlider("Launch Power")
      .setPosition(7 * WINDOW_WIDTH/10, row_2)
     .setSize(50,200)
     .setRange(0, 100)
     .setValue(0)
     .setColorBackground(UI_COLOR)
      .setColorValueLabel(FOREGROUND_COLOR)
      .setColorCaptionLabel(FOREGROUND_COLOR)

    ; 

     
  oscP5.plug(this,"setEndPitch","/end_pitch");   
  oscP5.plug(this,"setSwingRotation","/swing_rotation");   
  oscP5.plug(this,"setLaunchAngle","/launch_angle");      
  oscP5.plug(this,"setLaunchPower","/launch_power");  

  valueList = new ArrayList<String>();  
}

void draw() {
  background(color(BACKGROUND_COLOR));  
  
  textSize(28);
  textAlign(CENTER);
  fill(FOREGROUND_COLOR);
  text("GolfTrainer", WINDOW_WIDTH/2, 50);

  // Display the list of last 5 values
  displayValueTable();
}

void setEndPitch(float input_value) {
  end_pitch.setArrayValue(new float[] {input_value, 5}); 
}


void setSwingRotation(float input_value) {
  swing_rotation.setValue(input_value);   

  // Set the active color based on the value of the angle
  if (input_value > (80)) {
    swing_rotation.setColorForeground(RED);
  }  else {
    swing_rotation.setColorForeground(GREEN);
  }
}

void setLaunchPower(float input_value) {
  launch_power.setValue(input_value);   

    if (input_value > (MAX_SWING_HEIGHT-20)) {
    launch_power.setColorForeground(RED);
  }  else {
    launch_power.setColorForeground(GREEN);
  }
}

void setLaunchAngle(float input_value) {
  launch_angle.setValue(input_value); 
 
  // Set the active color based on the value of the angle
  if (input_value > (MAX_LAUNCH_ANGLE-20) || input_value < (MIN_LAUNCH_ANGLE+20)) {
    launch_angle.setColorForeground(RED);
  }  else {
    launch_angle.setColorForeground(GREEN);
  }

  if(input_value > 10) {
    text_box.setText("HIT! A little bit on the right");
  } else if(input_value < -10) {
    text_box.setText("HIT! A little bit on the left");
  } else {
    text_box.setText("HIT! Perfect launch!");
  }

  float launch_angle_v = launch_angle.getValue();
  float launch_power_v = launch_power.getValue();
  float end_pitch_v = end_pitch.getArrayValue()[0];
  addToList(end_pitch_v, launch_angle_v, launch_power_v); 
}

void addToList(float end_pitch, float launch_angle, float launch_power) {
  // Format the entry as a string
  String entry = round(end_pitch) + "," + round(launch_angle) + "," + round(launch_power);
  
  // Add the entry to the list
  valueList.add(entry);
  
  // If the list exceeds 5 entries, remove the oldest one
  if (valueList.size() > 5) {
    valueList.remove(0);
  }
}

void displayValueTable() {
  // Table headers
  fill(FOREGROUND_COLOR);
  textSize(20);
  textAlign(LEFT, TOP);


  float xPos = 600;  // Starting position for columns
  float yPos = row_2;  // Starting position for headers
  float columnSpacing = 100; // Adjust spacing between columns
  
  text("Last 5 launches", xPos, yPos);  // Display the title


  textSize(16);
  yPos += 30;
  // Display headers in columns
  text("End Pitch", xPos + 0 * columnSpacing, yPos);
  text("Launch Angle", xPos + 1 * columnSpacing, yPos);
  text("Launch Power", xPos + 2 * columnSpacing, yPos);

  yPos += 30;
    
  for (int i = 0; i < valueList.size(); i++) {
    String[] values = split(valueList.get(i), ',');  // Split the values by comma
    for (int j = 0; j < values.length; j++) {
      text(values[j], xPos + j * columnSpacing, yPos + i * 20);  // Display each entry in the table format
    }
  }
}

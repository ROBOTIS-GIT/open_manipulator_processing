/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */


/**
 * this code is compatible with open_manipulator_stewart.ino
**/

// Multiple Window
ChildApplet child;

// Control Interface
import controlP5.*;

// Init serial
import processing.serial.*;

// Shape variables
PShape base_shape;
PShape goal_link1_shape, goal_link2_shape, goal_link3_shape, 
       goal_link4_shape, goal_link5_shape, goal_link6_shape, 
       goal_link7_shape, goal_link8_shape, goal_link9_shape, 
       goal_link10_shape, goal_link11_shape, goal_link12_shape, 
       goal_tool_shape;
PShape ctrl_link1_shape, ctrl_link2_shape, ctrl_link3_shape, 
       ctrl_link4_shape, ctrl_link5_shape, ctrl_link6_shape, 
       ctrl_link7_shape, ctrl_link8_shape, ctrl_link9_shape, 
       ctrl_link10_shape, ctrl_link11_shape, ctrl_link12_shape, 
       ctrl_tool_shape;

// Model pose
float model_trans_x, model_trans_y, model_trans_z, model_scale_factor;

// World pose
float world_rot_x, world_rot_y;

// Serial variable
Serial opencr_port;

// Angle variable
float[] receive_joint_angle = new float[21];
float receive_tool_pos = 0.0;

float[] ctrl_joint_angle = new float[20];
float ctrl_tool_pos = 0.0;

int tabFlag = 1;

float[] visual_target_pose_x = new float[50];
float[] visual_target_pose_y = new float[50];
float[] visual_target_pose_z = new float[50];

/*******************************************************************************
* Setting window size
*******************************************************************************/
void settings()
{
  size(900, 900, OPENGL);
}

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  surface.setTitle("OpenManipulator Stewart");
  child = new ChildApplet();

  initShape();
  initView();

  connectOpenCR(0); // Inside the brackets depends on a laptop enviroment.
}

/*******************************************************************************
* Draw (loop function)
*******************************************************************************/
void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  drawManipulator();
}

/*******************************************************************************
* Connect OpenCR
*******************************************************************************/
void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');
}

/*******************************************************************************
* Serial Interrupt
*******************************************************************************/
void serialEvent(Serial opencr_port)
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  String[] cmd = split(opencr_string, ',');

  if (cmd[0].equals("joint"))
  {
    for (int cmd_cnt = 1; cmd_cnt < cmd.length; cmd_cnt++)
    {
      receive_joint_angle[cmd_cnt-1] = float(cmd[cmd_cnt]);
    }
  }
  else
  {
    println("Error");
  }
}

/*******************************************************************************
* Init viewpoint and camera
*******************************************************************************/
void initView()
{
  float camera_y = height/2.0;
  float fov = 200/float(width) * PI/2;
  float camera_z = camera_y / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, camera_z/10.0, camera_z*10.0);

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0-500, height/2.0 * 4,
         width/2, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Get shape
*******************************************************************************/
void initShape()
{

  base_shape = loadShape("meshes/base.obj");
  goal_link1_shape = loadShape("meshes/link1.obj");
  goal_link2_shape = loadShape("meshes/link1.obj");
  goal_link3_shape = loadShape("meshes/link1.obj");
  goal_link4_shape = loadShape("meshes/link1.obj");
  goal_link5_shape = loadShape("meshes/link1.obj");
  goal_link6_shape = loadShape("meshes/link1.obj");
  goal_link7_shape = loadShape("meshes/link2.obj");
  goal_link8_shape = loadShape("meshes/link2.obj");
  goal_link9_shape = loadShape("meshes/link2.obj");
  goal_link10_shape = loadShape("meshes/link2.obj");
  goal_link11_shape = loadShape("meshes/link2.obj");
  goal_link12_shape = loadShape("meshes/link2.obj");
  goal_tool_shape  = loadShape("meshes/tool.obj");

  ctrl_link1_shape = loadShape("meshes/link1.obj");
  ctrl_link2_shape = loadShape("meshes/link1.obj");
  ctrl_link3_shape = loadShape("meshes/link1.obj");
  ctrl_link4_shape = loadShape("meshes/link2.obj");
  ctrl_link5_shape = loadShape("meshes/link2.obj");
  ctrl_link6_shape = loadShape("meshes/link2.obj");
  ctrl_link7_shape = loadShape("meshes/link2.obj");
  ctrl_link8_shape = loadShape("meshes/link2.obj");
  ctrl_link9_shape = loadShape("meshes/link2.obj");
  ctrl_link10_shape = loadShape("meshes/link2.obj");
  ctrl_link11_shape = loadShape("meshes/link2.obj");
  ctrl_link12_shape = loadShape("meshes/link2.obj");
  ctrl_tool_shape  = loadShape("meshes/tool.obj");

  goal_link1_shape.setFill(color(200));
  goal_link2_shape.setFill(color(200));
  goal_link3_shape.setFill(color(200));
  goal_link4_shape.setFill(color(200));
  goal_link5_shape.setFill(color(200));
  goal_link6_shape.setFill(color(200));
  goal_link7_shape.setFill(color(200));
  goal_link8_shape.setFill(color(200));
  goal_link9_shape.setFill(color(200));
  goal_link10_shape.setFill(color(200));
  goal_link11_shape.setFill(color(200));
  goal_link12_shape.setFill(color(200));
  goal_tool_shape.setFill(color(200));

  ctrl_link1_shape.setFill(color(255,255,255));
  ctrl_link2_shape.setFill(color(255,255,255));
  ctrl_link3_shape.setFill(color(255,255,255));
  ctrl_link4_shape.setFill(color(255,255,255));
  ctrl_link5_shape.setFill(color(255,255,255));
  ctrl_link6_shape.setFill(color(255,255,255));
  ctrl_link7_shape.setFill(color(255,255,255));
  ctrl_link8_shape.setFill(color(255,255,255));
  ctrl_link9_shape.setFill(color(255,255,255));
  ctrl_link10_shape.setFill(color(255,255,255));
  ctrl_link11_shape.setFill(color(255,255,255));
  ctrl_link12_shape.setFill(color(255,255,255));
  ctrl_tool_shape.setFill(color(255,255,255));

  setJointAngle(0, 0, 0);
}

/*******************************************************************************
* Set window characteristic
*******************************************************************************/
void setWindow()
{
  lights();
  smooth();
  background(30);

  translate(width/2, height/2+150, 0);

  rotateX(radians(90));
  rotateZ(radians(140));
}

/*******************************************************************************
* Draw sphere
*******************************************************************************/
void drawSphere(int x, int y, int z, int r, int g, int b, int size)
{
  translate(x, y, z);
  stroke(r,g,b);
  sphere(size);
}

void saveSpherePose()
{
  for (int i = 0; i < visual_target_pose_x.length - 1; i++)
  {
    visual_target_pose_x[i] = visual_target_pose_x[i + 1];
    visual_target_pose_y[i] = visual_target_pose_y[i + 1];
    visual_target_pose_z[i] = visual_target_pose_z[i + 1];
  }

  visual_target_pose_x[visual_target_pose_x.length - 1] = modelX(0,0,0);
  visual_target_pose_y[visual_target_pose_y.length - 1] = modelY(0,0,0);
  visual_target_pose_z[visual_target_pose_z.length - 1] = modelZ(0,0,0);
}

void drawSphereAfterEffect()
{
  for (int i = 0; i < visual_target_pose_x.length; i++)
  { 
    pushMatrix();
    rotateZ(radians(90));
    rotateX(radians(0));
    translate(-width/2, -height/2-200, 0);
    
    translate(visual_target_pose_x[i], visual_target_pose_y[i], visual_target_pose_z[i]);
    stroke(255,255,255);
    sphere(1.5);
    popMatrix();
  }
}

/*******************************************************************************
* Draw title
*******************************************************************************/
void drawTitle()
{
  scale(1 + model_scale_factor);
  pushMatrix();
  rotateX(radians(80));
  rotateZ(radians(180));
  textSize(50);
  fill(255,204,102);
  text("OpenManipulator Stewart", -250,-385,0);
  textSize(25);
  fill(102,255,255);
  text("Move manipulator 'Q,A','W,S','E,D'", -250,-320,0);
  text("Initial view 'I'", -250,-280,0);
  popMatrix();
}

/*******************************************************************************
* Draw manipulator
*******************************************************************************/
void drawManipulator()
{
  scale(1.0 + model_scale_factor);

  // Base
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  shape(base_shape);
  popMatrix();

  // First Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(0.436);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(PI-receive_joint_angle[0]);
  shape(goal_link1_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  rotateX(-2.100 - receive_joint_angle[6]);
  rotateZ(0.064 + receive_joint_angle[7]);
  shape(goal_link7_shape);
      
  popMatrix();
    
    
    // First Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(-0.436);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(-receive_joint_angle[1]);
  shape(goal_link4_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  //drawLocalFrame();
  rotateX(2.100 - receive_joint_angle[8]);
  rotateZ(0.064 - receive_joint_angle[9]);
  shape(goal_link8_shape);

  popMatrix();

  //// Second Set
  // First Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(+0.436-2.094);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(PI-receive_joint_angle[2]);
  shape(goal_link5_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  rotateX(-2.100 - receive_joint_angle[10]);
  rotateZ(0.064 + receive_joint_angle[11]);
  shape(goal_link9_shape);

  popMatrix();
      
      
  //// Second Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(-0.436-2.094);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(-receive_joint_angle[3]);
  shape(goal_link5_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  rotateX(2.100 - receive_joint_angle[12]);
  rotateZ(0.064 - receive_joint_angle[13]);
  shape(goal_link10_shape);

  popMatrix();
    
  //// Third Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(+0.436-4.189);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(PI-receive_joint_angle[4]);
  shape(goal_link5_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  rotateX(-2.100 - receive_joint_angle[14]);
  rotateZ(0.064 + receive_joint_angle[15]);
  shape(goal_link11_shape);

  popMatrix();
      
  //// Third Set
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  rotateZ(-0.436-4.189);
  translate(0.070*1000, 0, 0.04025*1000);
  rotateX(-receive_joint_angle[5]);
  shape(goal_link5_shape);
  drawLocalFrame();

  translate(0.0074*1000, 0.026*1000, 0);
  rotateX(2.100 - receive_joint_angle[16]);
  rotateZ(0.064 - receive_joint_angle[17]);
  shape(goal_link12_shape);

  popMatrix();

  //// Tool
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateZ(PI);
  // rotateZ(-receive_joint_angle[7]);
  // rotateY(-receive_joint_angle[0] - receive_joint_angle[6] + 0.075);
  translate(-receive_joint_angle[18]*1000, receive_joint_angle[19]*1000, receive_joint_angle[20]*1000);
  shape(goal_tool_shape);

  popMatrix();
 
  // ////----------------------- Control
  // if (tabFlag == 1)
  // {
  //   // First Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(0.436);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(PI-ctrl_joint_angle[0]);
  //   shape(goal_link1_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   rotateX((-2.103914)+ctrl_joint_angle[6]);
  //   rotateZ(-(ctrl_joint_angle[7]-0.075));
  //   shape(goal_link7_shape);
      
  //   popMatrix();
    
    
  //   // First Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(-0.436);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(-ctrl_joint_angle[1]);
  //   shape(goal_link4_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   //drawLocalFrame();
  //   rotateX((+2.103914)+ctrl_joint_angle[8]);
  //   rotateZ(-(ctrl_joint_angle[9]-0.075));
  //   shape(goal_link8_shape);

  //   popMatrix();

  // //// Second Set
  //   // First Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(+0.436-2.094);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(PI-ctrl_joint_angle[2]);
  //   shape(goal_link5_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   rotateX((-2.103914)+ctrl_joint_angle[10]);
  //   rotateZ(-(ctrl_joint_angle[11]-0.075));
  //   shape(goal_link9_shape);

  //   popMatrix();
      
      
  // //// Second Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(-0.436-2.094);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(-ctrl_joint_angle[3]);
  //   shape(goal_link5_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   rotateX((2.103914)+ctrl_joint_angle[12]);
  //   rotateZ(-(ctrl_joint_angle[13]-0.075));
  //   shape(goal_link10_shape);

  //   popMatrix();
    
  //   //// Third Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(+0.436-4.189);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(PI-ctrl_joint_angle[4]);
  //   shape(goal_link5_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   rotateX((-2.103914)+ctrl_joint_angle[14]);
  //   rotateZ(-(ctrl_joint_angle[15]-0.075));
  //   shape(goal_link11_shape);

  //   popMatrix();
      
  //   //// Third Set
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   rotateZ(-0.436-4.189);
  //   translate(0.070*1000, 0, 0.04025*1000);
  //   rotateX(-ctrl_joint_angle[5]);
  //   shape(goal_link5_shape);
  //   drawLocalFrame();

  //   translate(0.0074*1000, 0.026*1000, 0);
  //   rotateX((2.103914)+ctrl_joint_angle[16]);
  //   rotateZ(-(ctrl_joint_angle[17]-0.075));
  //   shape(goal_link12_shape);

  //   popMatrix();


  //   //// Tool
  //   pushMatrix();
  //   translate(-model_trans_x, -model_trans_y, -model_trans_z);
  //   translate(ctrl_joint_angle[18]*1000, ctrl_joint_angle[19]*1000, 0.0*1000);
  //   shape(goal_tool_shape);

  //   popMatrix();
  // }
}

/*******************************************************************************
* Draw world frame
*******************************************************************************/
void drawWorldFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 500);
  line(0, 0, 0, 100, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -100, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 100);
}

/*******************************************************************************
* Draw local frame
*******************************************************************************/
void drawLocalFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 60, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -60, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 60);
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setJointAngle(float angle1, float angle2, float angle3)
{
  ctrl_joint_angle[0] = angle1;
  ctrl_joint_angle[1] = angle2;
  ctrl_joint_angle[2] = angle3;
}

/*******************************************************************************
* Gripper on
*******************************************************************************/
void gripperOn()
{
  ctrl_tool_pos = radians(0.0);
}

/*******************************************************************************
* Gripper off
*******************************************************************************/
void gripperOff()
{
  ctrl_tool_pos = radians(-1.0);
}

/*******************************************************************************
* Mouse drag event
*******************************************************************************/
void mouseDragged()
{
  world_rot_x -= (mouseX - pmouseX) * 2.0;
  world_rot_y -= (mouseY - pmouseY) * 2.0;

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0 + world_rot_x, height/2.0 + world_rot_y, height/2.0 * 4,
         width/2, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Mouse wheel event
*******************************************************************************/
void mouseWheel(MouseEvent event) {
  float e = event.getCount() * 0.01;
  model_scale_factor += e;
}

/*******************************************************************************
* Key press event
*******************************************************************************/
void keyPressed()
{
  if      (key == 'q') model_trans_x      -= 0.0050 * 1000;
  else if (key == 'a') model_trans_x      += 0.0050 * 1000;
  else if (key == 'w') model_trans_y      += 0.0050 * 1000;
  else if (key == 's') model_trans_y      -= 0.0050 * 1000;
  else if (key == 'e') model_trans_z      -= 0.0050 * 1000;
  else if (key == 'd') model_trans_z      += 0.0050 * 1000;
  else if (key == 'i') 
  {
    model_trans_x = model_trans_y = model_trans_z = model_scale_factor = world_rot_x = world_rot_y = 0.0;
    camera(width/2.0, height/2.0-500, height/2.0 * 4,
           width/2, height/2, 0,
           0, 1, 0);
  }
}

/*******************************************************************************
* Controller Window
*******************************************************************************/
class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel headLabel;
  Knob joint1, joint2, joint3, joint4, joint5, joint6, tool;
  Slider2D slider2d;

  float[] set_joint_angle = new float[3];
  float set_tool_pos;

  boolean onoff_flag = false;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 600);
    smooth();
  }
  public void setup()
  {
    surface.setTitle("Control Interface");
    cp5 = new ControlP5(this);

/*******************************************************************************
* Init Tab
*******************************************************************************/
    cp5.addTab("Motion")
       .setColorBackground(color(0, 160, 100))
       .setColorLabel(color(255))
       .setColorActive(color(0,0,255))
       ;

    cp5.getTab("default")
       .activateEvent(true)
       .setLabel("Task Space Control")
       .setId(1)
       ;

    cp5.getTab("Motion")
       .activateEvent(true)
       .setId(2)
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    headLabel = cp5.addTextlabel("Label")
                   .setText("Controller for OpenManipulator Stewart")
                   .setPosition(4,17)
                   .setColorValue(0xffffff00)
                   .setFont(createFont("arial",20))
                   ;

    cp5.addToggle("Controller_OnOff")
       .setCaptionLabel("      Controller Off             Controller On")
       .setPosition(0,50)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

    cp5.addButton("Forward")
       .setValue(0)
       .setPosition(150,150)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Back")
       .setValue(0)
       .setPosition(150,350)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Left")
       .setValue(0)
       .setPosition(50,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Right")
       .setValue(0)
       .setPosition(250,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Up")
       .setValue(0)
       .setPosition(50,450)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Down")
      .setValue(0)
      .setPosition(250,450)
      .setSize(100,100)
      .setFont(createFont("arial",15))
      ;

    cp5.addButton("Set")
       .setCaptionLabel("Origin")
       .setValue(0)
       .setPosition(170,270)
       .setSize(60,60)
       .setFont(createFont("arial",15))
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    cp5.addButton("Motion_Start")
       .setCaptionLabel("Motion Start")
       .setValue(0)
       .setPosition(0,200)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Motion_Stop")
       .setCaptionLabel("Motion Stop")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

/*******************************************************************************
* Set Tap UI
*******************************************************************************/
    cp5.getController("Label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    cp5.getController("Motion_Start").moveTo("Motion");
    cp5.getController("Motion_Stop").moveTo("Motion");
  }

  public void draw()
  {
    background(0);
  }

  void controlEvent(ControlEvent theControlEvent) {
    if (theControlEvent.isTab()) {
      println("got an event from tab : "+theControlEvent.getTab().getName()+" with id "+theControlEvent.getTab().getId());
      tabFlag = theControlEvent.getTab().getId();
    }
  }

/*******************************************************************************
* Init Function of Task Space Controller
*******************************************************************************/
  void Controller_OnOff(boolean flag)
  {
    onoff_flag = flag;
    if (onoff_flag)
    {
      joint1.setValue(ctrl_joint_angle[0]);
      joint2.setValue(ctrl_joint_angle[1]);
      joint3.setValue(ctrl_joint_angle[2]);
      joint3.setValue(ctrl_joint_angle[2]);
      joint3.setValue(ctrl_joint_angle[4]);
      joint3.setValue(ctrl_joint_angle[5]);
      joint3.setValue(ctrl_joint_angle[6]);
      tool.setValue(ctrl_tool_pos);

      opencr_port.write("actuator"   + ',' +
                        "on" + '\n');
      println("OpenManipulator Stewart Ready!!!");
    }
    else
    {
      opencr_port.write("actuator"  + ',' +
                        "off"  + '\n');
      println("OpenManipulator Stewart End...");
    }
  }

  public void Forward(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "f" + '\n');
      println("Move Forward");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Back(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "b"    + '\n');
      println("Move Back");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Left(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "l"    + '\n');
      println("Move Left");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Right(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "r"   + '\n');
      println("Move Right");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Up(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "u"      + '\n');
      println("Move Up");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Down(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "d"    + '\n');
      println("Move Down");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        " "       + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Motion
*******************************************************************************/
  public void Motion_Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("demo"  + ',' +
                        "start"   + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Motion_Stop(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("demo"  + ',' +
                        "stop"    + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
}



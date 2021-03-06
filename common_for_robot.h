/****** controll modes  **************/
#define DRIVE_ONLY      0
#define R_ARM_ONLY      1
#define L_ARM_ONLY      2
#define R_ARM_AND_DRIVE 3
#define L_ARM_AND_DRIVE 4
#define DEFAULT_MODE    100

/******Timing defines*************/
#define CONN_LOST_TIME    3000  //3 s

const String alive = "X";
const String modS = "Y";
const String controlSign = "@";
const String ack = "Z";
String inputDataIDArray[] = {"A",//"rF2", //Right front 2
							 "B",//"rF1", //Right front 1
							 "C",//"lF2", //Left front 2
							 "D",//"lF1", //Left front 2
							 "E",//"bu4", //Button 4
							 "F",//"bu3", //Button 3
							 "G",//"bu2", //Button 2
							 "H",//"bu1", //Button 1
							 "I",//"up0", //Up arrow
							 "J",//"rig", //Right arrow
							 "K",//"lef", //Left arrow
							 "L",//"dow", //Down arrow
							 "M",//"rJX", //Right joystic X
							 "N",//"rJY", //Right joystic Y
							 "O",//"lJX", //Left joystic X
							 "P"};//"lJY"};//Left joystic Y

#include <tinyFAT.h>
#include <UTFT.h>
#include <UTFT_tinyFAT.h>
#include <i2cmaster.h>
#include <Servo.h>


//-----------------THC--------------------------------------------------------------------
#define HORIZ_SERVO_PIN 8
#define VERT_SERVO_PIN 9
//This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
	double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
	double tempData = 0x0000; // zero out the data
int dev = 0x5A<<1;
int data_low = 0;
int data_high = 0;
int pec = 0;
long int count = 0;
int mlr = 1275; //Middle point for LR Servo; only used if there is no existing calibration
int mud = 1750; //Middle point for UD Servo; only used if there is no existing calibration
int blr = 1600; //Left-Bottom point for LR Servo; only used if there is no existing calibration
int bud = 1550; //Left-Bottom point for UD Servo; only used if there is no existing calibration
int lines = 6;
int rows = 8;
int rawTemperaturesLine100[48]; //int = 36.6 *100 = 3660
Servo lr; //Servo for left-right movement
Servo ud; //Servo for up-down movement
//-------------------------TFT-------------------------------------------------
UTFT        myGLCD(TFT01_32,38,39,40,41); 
byte gradientArray[101][3] = {
	{ 
		5, 5, 190   }
	,{ 
		13, 2, 178   }
	,{ 
		14, 0, 175   }
	,{ 
		18, 0, 172   }
	,{ 
		22, 0, 169   }
	,{ 
		26, 0, 166   }
	,{ 
		30, 0, 163   }
	,{ 
		35, 0, 160   }
	,{ 
		40, 0, 156   }
	,{ 
		45, 0, 152   }
	,{ 
		51, 0, 149   }
	,{ 
		56, 0, 144   }
	,{ 
		62, 0, 140   }
	,{ 
		68, 0, 136   }
	,{ 
		75, 0, 131   }
	,{ 
		81, 0, 126   }
	,{ 
		88, 0, 122   }
	,{ 
		94, 0, 117   }
	,{ 
		101, 0, 112   }
	,{ 
		108, 0, 107   }
	,{ 
		114, 0, 102   }
	,{ 
		121, 0, 97   }
	,{ 
		128, 0, 92   }
	,{ 
		135, 0, 87   }
	,{ 
		141, 0, 83   }
	,{ 
		147, 0, 79   }
	,{ 
		152, 0, 75   }
	,{ 
		158, 0, 70   }
	,{ 
		164, 0, 66   }
	,{ 
		170, 0, 62   }
	,{ 
		175, 0, 58   }
	,{ 
		181, 0, 54   }
	,{ 
		186, 0, 50   }
	,{ 
		192, 0, 46   }
	,{ 
		197, 0, 42   }
	,{ 
		202, 0, 38   }
	,{ 
		207, 0, 35   }
	,{ 
		212, 0, 31   }
	,{ 
		217, 0, 28   }
	,{ 
		221, 0, 24   }
	,{ 
		226, 0, 21   }
	,{ 
		230, 0, 18   }
	,{ 
		234, 0, 15   }
	,{ 
		238, 0, 12   }
	,{ 
		242, 0, 10   }
	,{ 
		245, 0, 7   }
	,{ 
		248, 0, 5   }
	,{ 
		251, 0, 3   }
	,{ 
		254, 0, 1   }
	,{ 
		255, 1, 0   }
	,{ 
		255, 3, 0   }
	,{ 
		255, 5, 0   }
	,{ 
		255, 8, 0   }
	,{ 
		255, 10, 0   }
	,{ 
		255, 13, 0   }
	,{ 
		255, 16, 0   }
	,{ 
		255, 19, 0   }
	,{ 
		255, 22, 0   }
	,{ 
		255, 25, 0   }
	,{ 
		255, 28, 0   }
	,{ 
		255, 32, 0   }
	,{ 
		255, 35, 0   }
	,{ 
		255, 39, 0   }
	,{ 
		255, 43, 0   }
	,{ 
		255, 46, 0   }
	,{ 
		255, 51, 0   }
	,{ 
		255, 55, 0   }
	,{ 
		255, 59, 0   }
	,{ 
		255, 63, 0   }
	,{ 
		255, 67, 0   }
	,{ 
		255, 72, 0   }
	,{ 
		255, 76, 0   }
	,{ 
		255, 80, 0   }
	,{ 
		255, 85, 0   }
	,{ 
		255, 90, 0   }
	,{ 
		255, 94, 0   }
	,{ 
		255, 99, 0   }
	,{ 
		255, 103, 0   }
	,{ 
		255, 108, 0   }
	,{ 
		255, 113, 0   }
	,{ 
		255, 118, 0   }
	,{ 
		255, 122, 0   }
	,{ 
		255, 128, 0   }
	,{ 
		255, 136, 0   }
	,{ 
		255, 145, 0   }
	,{ 
		255, 153, 0   }
	,{ 
		255, 161, 0   }
	,{ 
		255, 170, 0   }
	,{ 
		255, 178, 0   }
	,{ 
		255, 185, 0   }
	,{ 
		255, 193, 0   }
	,{ 
		255, 200, 0   }
	,{ 
		255, 207, 0   }
	,{ 
		255, 214, 0   }
	,{ 
		255, 220, 0   }
	,{ 
		255, 227, 0   }
	,{ 
		255, 232, 0   }
	,{ 
		255, 237, 0   }
	,{ 
		255, 242, 0   }
	,{ 
		255, 247, 0   }
	, { 
		255, 252, 17   } 
};
extern uint8_t SmallFont[];
//--------------------------SD--------------------------------------------------
byte sdRes;
word sdResult;
char sdTextBuffer[81];
char newFileNameChars[] = "T999.THC";
char currFileNameChars[] = "T999.THC";
//-------------------------render-------------------------------------------------
int renderTemperature;
int renderPixelColorIndex;
int renderMaxT=-10000;
int renderMinT =10000;
///////---------------------------------------------------------------------------------------------------------------

int getRawTemperature100(){
	i2c_start_wait(dev+I2C_WRITE);
	i2c_write(0x07);	
	// read
	i2c_rep_start(dev+I2C_READ);
	data_low = i2c_readAck(); //Read 1 byte and then send ack
	data_high = i2c_readAck(); //Read 1 byte and then send ack
	pec = i2c_readNak();
	i2c_stop();	
	
	// This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
	tempData = (double)(((data_high & 0x007F) << 8) + data_low);
	tempData = (tempData * tempFactor)-0.01;
	
	int celcius = (tempData - 273.15)*100;
	delay(60);
	return celcius;
}


void scan() {
	// Raw temperature values. This size must be less than lines.
	int rawValues[128];
	int yPos = bud;
	int yInc = ((mud - bud) * 2) / (lines - 1);
	int xPos = blr;
	int xInc = ((blr - mlr) * 2) / (rows - 1);

	
	sdRes=file.initFAT();
	if (file.exists(newFileNameChars)) file.delFile(newFileNameChars);	
	file.create(newFileNameChars);
	delay(100);  
	myGLCD.print(newFileNameChars,CENTER, 184);

	ud.writeMicroseconds(yPos);
	lr.writeMicroseconds(xPos);
	for (int x = 0; x < rows; x++) { 	    
		for (int y = 0; y < lines; y++) {
			rawTemperaturesLine100[y]	= getRawTemperature100();
			if (y != lines - 1) {
				yPos += yInc;
				ud.writeMicroseconds(yPos);        
			}
		}
		if (x != rows - 1) {
			xPos -= xInc;
			lr.writeMicroseconds(xPos);
		}
		// Move fairly quickly so that we don't scan the latent temperature of objects in the middle.
		// Don't move too fast to cause a lot of jiggle.
		for (int i = 0; i < (lines - 1); i++) {
			yPos -= yInc;
			ud.writeMicroseconds(yPos);
			delay(2);
		}    
		saveTemperaturePixelToSD(rawValues, lines);
	}  
}


void saveTemperaturePixelToSD(int *rawValues, int length){	
	char temperatureChars[32];      	
	sdRes=file.openFile(newFileNameChars, FILEMODE_TEXT_WRITE);
	for (int idx = 0; idx < length; idx++) {
		itoa(rawValues[idx],temperatureChars,DEC);    
		if (sdRes==NO_ERROR)
		{    
			file.writeLn(temperatureChars);      		
		}       
	}	
	if (sdRes==NO_ERROR)
	{   		
		file.closeFile();
	}	
	myGLCD.print("saveTemperaturePixelToSD",CENTER, 144);
}



void makeNewFilename(){
	int newFileNameIndex = 1;
	sdRes=file.initFAT();
	if (file.exists("FILES.SYS"))
	{  
		sdRes=file.openFile("FILES.SYS", FILEMODE_TEXT_READ);
		if (sdRes==NO_ERROR)
		{
			sdResult=0;
			//while ((sdResult!=EOF) and (sdResult!=FILE_IS_EMPTY)) {
			sdResult=file.readLn(sdTextBuffer, 80);
			newFileNameIndex += atoi(sdTextBuffer);				
			newFileNameChars[0] = 'T'; //(newFileNameIndex/1000)%10 + '0'; 
			newFileNameChars[1] = (newFileNameIndex/100)%10 + '0'; 
			newFileNameChars[2] = (newFileNameIndex/10)%10 + '0'; 
			newFileNameChars[3] = newFileNameIndex%10 + '0';  
			newFileNameChars[4] = '.';
			newFileNameChars[5] = 'T';
			newFileNameChars[6] = 'H';	
			newFileNameChars[7] = 'C';			
			//}			
			file.closeFile();
		}		
	}
	sdRes=file.initFAT(); 
	if (file.exists("FILES.SYS")){
		file.delFile("FILES.SYS");	
	}
	file.create("FILES.SYS");
	sdRes=file.openFile("FILES.SYS", FILEMODE_TEXT_WRITE);
	if (sdRes==NO_ERROR)
	{   
		char newFileNameIndexChars[] = "0000";
		itoa(newFileNameIndex,newFileNameIndexChars,DEC);
		file.writeLn(newFileNameIndexChars);		
		file.closeFile();
	}		
}



void renderFindMaxMinT(){		
	sdRes=file.initFAT();
	if (file.exists(currFileNameChars))
	{  
		sdRes=file.openFile(currFileNameChars, FILEMODE_TEXT_READ);
		if (sdRes==NO_ERROR)
		{
			sdResult=0;
			while ((sdResult!=EOF) and (sdResult!=FILE_IS_EMPTY))
			{
				sdResult=file.readLn(sdTextBuffer, 80);
				if (sdResult!=FILE_IS_EMPTY)
				{
					renderTemperature = atof((char*)sdTextBuffer);		  
					if(renderMaxT < renderTemperature){ 
						renderMaxT = renderTemperature; 
					}
					if(renderMinT > renderTemperature){ 
						renderMinT = renderTemperature; 
					}           
				}        
			}      
			file.closeFile();
		}    
	} 	   
}

int renderConvertTemperatureToColorPos(int renderTemperature){
	return constrain(map(renderTemperature, renderMinT, renderMaxT, 0, 99),0,99); 
}

void renderDrawPixelTemperature(int renderTemperature, int x, int y){
	renderPixelColorIndex = renderConvertTemperatureToColorPos(renderTemperature);     
	myGLCD.setColor(gradientArray[renderPixelColorIndex][0], gradientArray[renderPixelColorIndex][1], gradientArray[renderPixelColorIndex][2]);
	//myGLCD.drawPixel(x,y);
	myGLCD.fillRect(x*5,y*5,x*5+4,y*5+4);
}

void renderResult(){	
	int x=0;
	int y=47;
	sdRes=file.initFAT();
	if (file.exists(currFileNameChars))
	{  
		sdRes=file.openFile(currFileNameChars, FILEMODE_TEXT_READ);
		if (sdRes==NO_ERROR)
		{
			sdResult=0;
			while ((sdResult!=EOF) and (sdResult!=FILE_IS_EMPTY))
			{
				sdResult=file.readLn(sdTextBuffer, 80);
				if (sdResult!=FILE_IS_EMPTY)
				{
					renderTemperature = atof((char*)sdTextBuffer);		  
					renderDrawPixelTemperature(renderTemperature,x,y);
					y--;
					if(y<0) { 
						x++; 
						y=47; 
					}     

				}        
			}      
			file.closeFile();
		}    
	}
	myGLCD.setColor(240,240,240);	
	myGLCD.print(currFileNameChars,CENTER, 228);
}

void setup()
{  		  
	myGLCD.InitLCD();
	myGLCD.clrScr();     
	myGLCD.setBackColor(0, 0, 0);	  
	myGLCD.setFont(SmallFont);

    i2c_init();
	ud.attach(VERT_SERVO_PIN); //Attach servos
	lr.attach(HORIZ_SERVO_PIN); 
	ud.writeMicroseconds(mud); //Move servos to middle position
	lr.writeMicroseconds(mlr);

	//main process
	makeNewFilename();	
	delay(500);
	myGLCD.setColor(240,240,240);	
	myGLCD.print("begin scan",CENTER, 54);
	scan();  
	myGLCD.print("end scan",CENTER, 54);
	strcpy(currFileNameChars,newFileNameChars);	
	renderFindMaxMinT();
	renderResult();		
}



void loop()
{ 
	//delay(1000);
}


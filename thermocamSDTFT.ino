#include <UTFT.h>
#include <tinyFAT.h>
#include <avr/pgmspace.h>
UTFT        myGLCD(TFT01_32,38,39,40,41); 
int gradientArray[100][3] = {{ 13, 2, 178 },{ 14, 0, 175 },{ 18, 0, 172 },{ 22, 0, 169 },{ 26, 0, 166 },{ 30, 0, 163 },{ 35, 0, 160 },{ 40, 0, 156 },{ 45, 0, 152 },{ 51, 0, 149 },{ 56, 0, 144 },{ 62, 0, 140 },{ 68, 0, 136 },{ 75, 0, 131 },{ 81, 0, 126 },{ 88, 0, 122 },{ 94, 0, 117 },{ 101, 0, 112 },{ 108, 0, 107 },{ 114, 0, 102 },{ 121, 0, 97 },{ 128, 0, 92 },{ 135, 0, 87 },{ 141, 0, 83 },{ 147, 0, 79 },{ 152, 0, 75 },{ 158, 0, 70 },{ 164, 0, 66 },{ 170, 0, 62 },{ 175, 0, 58 },{ 181, 0, 54 },{ 186, 0, 50 },{ 192, 0, 46 },{ 197, 0, 42 },{ 202, 0, 38 },{ 207, 0, 35 },{ 212, 0, 31 },{ 217, 0, 28 },{ 221, 0, 24 },{ 226, 0, 21 },{ 230, 0, 18 },{ 234, 0, 15 },{ 238, 0, 12 },{ 242, 0, 10 },{ 245, 0, 7 },{ 248, 0, 5 },{ 251, 0, 3 },{ 254, 0, 1 },{ 255, 1, 0 },{ 255, 3, 0 },{ 255, 5, 0 },{ 255, 8, 0 },{ 255, 10, 0 },{ 255, 13, 0 },{ 255, 16, 0 },{ 255, 19, 0 },{ 255, 22, 0 },{ 255, 25, 0 },{ 255, 28, 0 },{ 255, 32, 0 },{ 255, 35, 0 },{ 255, 39, 0 },{ 255, 43, 0 },{ 255, 46, 0 },{ 255, 51, 0 },{ 255, 55, 0 },{ 255, 59, 0 },{ 255, 63, 0 },{ 255, 67, 0 },{ 255, 72, 0 },{ 255, 76, 0 },{ 255, 80, 0 },{ 255, 85, 0 },{ 255, 90, 0 },{ 255, 94, 0 },{ 255, 99, 0 },{ 255, 103, 0 },{ 255, 108, 0 },{ 255, 113, 0 },{ 255, 118, 0 },{ 255, 122, 0 },{ 255, 128, 0 },{ 255, 136, 0 },{ 255, 145, 0 },{ 255, 153, 0 },{ 255, 161, 0 },{ 255, 170, 0 },{ 255, 178, 0 },{ 255, 185, 0 },{ 255, 193, 0 },{ 255, 200, 0 },{ 255, 207, 0 },{ 255, 214, 0 },{ 255, 220, 0 },{ 255, 227, 0 },{ 255, 232, 0 },{ 255, 237, 0 },{ 255, 242, 0 },{ 255, 247, 0 }, { 255, 252, 17 } };

extern uint8_t SmallFont[];
byte res;
word result;
char textBuffer[81];
int temperature;
long pixelColorIndex;
int maxT=-10000;
int minT =10000;
int x=0;
int y=0;
String newFileName = "new.thc";
char newFileNameChar[] = "new.thc";


void createTemperatureFile(){
	res=file.initFAT(); 
	newFileName = String(random(10000,99999),DEC);
        newFileName += ".thc";
        newFileName.toCharArray(newFileNameChar,newFileName.length());
	file.create(newFileNameChar);
	res=file.openFile(newFileNameChar, FILEMODE_TEXT_WRITE);
	if (res==NO_ERROR)
	{		
		file.writeLn("01234567890123452345678901234567890123456789012345");			
		file.closeFile();
	}
}

void setMaxMinT(){		
	res=file.initFAT();
	if (file.exists("0.th"))
	{  
		res=file.openFile("0.th", FILEMODE_TEXT_READ);
		if (res==NO_ERROR)
		{
			result=0;
			while ((result!=EOF) and (result!=FILE_IS_EMPTY))
			{
				result=file.readLn(textBuffer, 80);
				if (result!=FILE_IS_EMPTY)
				{
					temperature = atof((char*)textBuffer)*100;		  
					if(maxT < temperature){ maxT = temperature; }
					if(minT > temperature){ minT = temperature; }           
				}        
			}      
			file.closeFile();
		}    
	} 	   
}

int convertTemperatureToColorPos(int temperature){
	return map(temperature, minT, maxT, 0, 99); 
}

void drawPixelTemperature(int temperature, int x, int y){
	pixelColorIndex = convertTemperatureToColorPos(temperature);     
	myGLCD.setColor(gradientArray[pixelColorIndex][0], gradientArray[pixelColorIndex][1], gradientArray[pixelColorIndex][2]);
	//myGLCD.drawPixel(x,y);
	myGLCD.fillRect(x*5,y*5,x*5+5,y*5+5);
}

void drawResult(){	
	int x=0;
	int y=47;
	res=file.initFAT();
	if (file.exists("0.th"))
	{  
		res=file.openFile("0.th", FILEMODE_TEXT_READ);
		if (res==NO_ERROR)
		{
			result=0;
			while ((result!=EOF) and (result!=FILE_IS_EMPTY))
			{
				result=file.readLn(textBuffer, 80);
				if (result!=FILE_IS_EMPTY)
				{
					temperature = atof((char*)textBuffer)*100;		  
					drawPixelTemperature(temperature,x,y);
					y--;
					if(y<0) { x++; y=47; }     
					
				}        
			}      
			file.closeFile();
		}    
	}	
}


void setup()
{  	
	randomSeed(analogRead(0));
	myGLCD.InitLCD();
	myGLCD.clrScr();     
	myGLCD.setBackColor(0, 0, 0);	  

	setMaxMinT();
	drawResult();
    createTemperatureFile();	
}



void loop()
{ 
  delay(1000);
}

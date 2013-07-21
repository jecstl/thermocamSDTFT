#include <tinyFAT.h>
#include <UTFT.h>
#include <UTFT_tinyFAT.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <inttypes.h>
#include <compat/twi.h>
#include <Servo.h>
#include <EEPROM.h>

//-----------------THC--------------------------------------------------------------------
#define HORIZ_SERVO_PIN 6
#define VERT_SERVO_PIN 7
#define MIN_READ_WAIT 60
#define SMB_READ    1
#define SMB_WRITE   0
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define SCL_CLOCK 200000L
long int count = 0;
int mlr = 1275; //Middle point for LR Servo; only used if there is no existing calibration
int mud = 1750; //Middle point for UD Servo; only used if there is no existing calibration
int blr = 1600; //Left-Bottom point for LR Servo; only used if there is no existing calibration
int bud = 1550; //Left-Bottom point for UD Servo; only used if there is no existing calibration
int lines = 48;
int rows = 64;
Servo lr; //Servo for left-right movement
Servo ud; //Servo for up-down movement
#define TEMP_OFFSET 27315
enum {
	TEMP_AMBIENT = 0x6,
	TEMP_OBJECT = 0x7
};
/*A raw temperature change larger than this requires more time for the sensor to warm up or cool down for an accurate reading.*/
#define MAX_SAFE_TEMPERATURE_CHANGE 50
//-------------------------TFT-------------------------------------------------
UTFT        myGLCD(TFT01_32,38,39,40,41); 
byte gradientArray[101][3] = {{ 5, 5, 190 },{ 13, 2, 178 },{ 14, 0, 175 },{ 18, 0, 172 },{ 22, 0, 169 },{ 26, 0, 166 },{ 30, 0, 163 },{ 35, 0, 160 },{ 40, 0, 156 },{ 45, 0, 152 },{ 51, 0, 149 },{ 56, 0, 144 },{ 62, 0, 140 },{ 68, 0, 136 },{ 75, 0, 131 },{ 81, 0, 126 },{ 88, 0, 122 },{ 94, 0, 117 },{ 101, 0, 112 },{ 108, 0, 107 },{ 114, 0, 102 },{ 121, 0, 97 },{ 128, 0, 92 },{ 135, 0, 87 },{ 141, 0, 83 },{ 147, 0, 79 },{ 152, 0, 75 },{ 158, 0, 70 },{ 164, 0, 66 },{ 170, 0, 62 },{ 175, 0, 58 },{ 181, 0, 54 },{ 186, 0, 50 },{ 192, 0, 46 },{ 197, 0, 42 },{ 202, 0, 38 },{ 207, 0, 35 },{ 212, 0, 31 },{ 217, 0, 28 },{ 221, 0, 24 },{ 226, 0, 21 },{ 230, 0, 18 },{ 234, 0, 15 },{ 238, 0, 12 },{ 242, 0, 10 },{ 245, 0, 7 },{ 248, 0, 5 },{ 251, 0, 3 },{ 254, 0, 1 },{ 255, 1, 0 },{ 255, 3, 0 },{ 255, 5, 0 },{ 255, 8, 0 },{ 255, 10, 0 },{ 255, 13, 0 },{ 255, 16, 0 },{ 255, 19, 0 },{ 255, 22, 0 },{ 255, 25, 0 },{ 255, 28, 0 },{ 255, 32, 0 },{ 255, 35, 0 },{ 255, 39, 0 },{ 255, 43, 0 },{ 255, 46, 0 },{ 255, 51, 0 },{ 255, 55, 0 },{ 255, 59, 0 },{ 255, 63, 0 },{ 255, 67, 0 },{ 255, 72, 0 },{ 255, 76, 0 },{ 255, 80, 0 },{ 255, 85, 0 },{ 255, 90, 0 },{ 255, 94, 0 },{ 255, 99, 0 },{ 255, 103, 0 },{ 255, 108, 0 },{ 255, 113, 0 },{ 255, 118, 0 },{ 255, 122, 0 },{ 255, 128, 0 },{ 255, 136, 0 },{ 255, 145, 0 },{ 255, 153, 0 },{ 255, 161, 0 },{ 255, 170, 0 },{ 255, 178, 0 },{ 255, 185, 0 },{ 255, 193, 0 },{ 255, 200, 0 },{ 255, 207, 0 },{ 255, 214, 0 },{ 255, 220, 0 },{ 255, 227, 0 },{ 255, 232, 0 },{ 255, 237, 0 },{ 255, 242, 0 },{ 255, 247, 0 }, { 255, 252, 17 } };
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


/**
* Initialization of the SMBus interface. Need to be called only once
* @param enablePullups Set to true to enable the internal pullups
*/
void smb_init(bool enablePullups) {
	if (enablePullups) {
		PORTC = (1 << PORTC4) | (1 << PORTC5); //Enabling Pull-Ups for the MLX90614
	}

	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	TWSR = 0;                         /* no prescaler */
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
}

/**
* Terminates the data transfer and releases the SMBus bus
*/
void smb_stop(void) {
	/* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while (TWCR & (1<<TWSTO)) {
	}
}

/**
* Wait until previous transmission completed
*/
static inline void smb_waitForRelease(void) {
	while (!(TWCR & (1<<TWINT))) {
	}
}

/**
* Send one byte to SMBus device
* @param data The byte to be transfered
* @return 0 write successful. 1 write failed
*/
uint8_t smb_write(uint8_t data) {	
	// Send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	smb_waitForRelease();

	// check value of TWI Status Register. Mask prescaler bits
	return ((TW_STATUS & 0xF8) != TW_MT_DATA_ACK);
}

/**
* Read one byte from the SMBus device, read is followed by a stop condition 
* @param arr The array to fill up.
* @param len The length of the array and the amount to read.
*/
void smb_read(uint8_t *arr, int len) {
	// Leave room for the last byte.
	len--;
	for (int idx = 0; idx < len; idx++) {
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
		smb_waitForRelease();
		arr[idx] = TWDR;
	}
	// Read last byte
	TWCR = (1<<TWINT) | (1<<TWEN);
	smb_waitForRelease();
	arr[len] = TWDR;
}

/**
* Issue a start condition, and return the status of operation
* @return 0 = device accessible, 1 = failed to access device
*/
static inline bool smb_startCondition() {
	// Send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	smb_waitForRelease();

	// Check value of TWI status register. Mask prescaler bits.
	uint8_t retVal = TW_STATUS & 0xF8;
	return (retVal != TW_START) && (retVal != TW_REP_START);
}

/**
* Write the address and transfer direction
* @param address The address and transfer direction of SMBus device
* @return The masked TW status
*/
static inline uint8_t smb_startWriteAddress(uint8_t address) {
	// Send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	smb_waitForRelease();

	// Check value of TWI Status Register. Mask prescaler bits.
	return TW_STATUS & 0xF8;
}

/**
* Issues a start condition and sends address and transfer direction.
* @param address The address and transfer direction of SMBus device
* @return 0 = device accessible, 1= failed to access device
*/
bool smb_start(uint8_t address) {
	if (smb_startCondition()) {
		return 1;
	}

	uint8_t twst = smb_startWriteAddress(address);
	return ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK));
}

/**
* Issues a start condition and sends address and transfer direction.
* If device is busy, use ack polling to wait until device is ready
* 
* @param address Address and transfer direction of SMBus device
*/
void smb_startWait(uint8_t address) {
	for (;;) {
		if (smb_startCondition()) {
			continue;
		}

		uint8_t twst = smb_startWriteAddress(address);
		if ((twst != TW_MT_SLA_NACK) && (twst != TW_MR_DATA_NACK)) {
			break;
		}
		/* The device is busy. Send stop condition to terminate write operation */
		smb_stop();
	}
}

/**
* Read the MLX90614 sensor using the SMBus.
* @param tempType Set to TEMP_AMBIENT or TEMP_OBJECT depending on the type of temperature you want to measure.
* @return Returns the temperature in Kelvin x 50. To get actual Kelvin, divide by 50.
*/
int readRawTemperature(uint8_t tempType) {
	uint8_t values[3];
	static const uint8_t DEVICE = 0x5A<<1;

	smb_startWait(DEVICE+SMB_WRITE);
	smb_write(tempType);
	smb_start(DEVICE+SMB_READ);
	smb_read(values, sizeof(values));
	// TODO: Use the 3rd byte (PEC) to verify the checksum. When it's invalid, requery the value.

	// Return temperature value. The data is sent as little endian.
	return ((int *)values)[0];
}



void scan() {
	// Raw temperature values. This size must be less than lines.
	int rawValues[128];
	int yPos = bud;
	int yInc = ((mud - bud) * 2) / (lines - 1);
	int xPos = blr;
	int xInc = ((blr - mlr) * 2) / (rows - 1);
	int lastRawVal = readRawTemperature(TEMP_OBJECT);

	ud.writeMicroseconds(yPos);
	lr.writeMicroseconds(xPos);
	for (int x = 0; x < rows; x++) {
		// Pause for a moment for the temperature reading to settle down in the new position.
		lastRawVal = readRawTemperature(TEMP_OBJECT);
		for (int msWait = MIN_READ_WAIT; msWait > 0 && lastRawVal == readRawTemperature(TEMP_OBJECT); msWait--) {
			delay(1);
		}
		// Move a little away from the edge of when the reading is updated.
		delay(MIN_READ_WAIT / 5 + MIN_READ_WAIT);
		lastRawVal = readRawTemperature(TEMP_OBJECT);
		delay(MIN_READ_WAIT);
		for (int y = 0; y < lines; y++) {
			
			// Read and save sensor object temperature
			rawValues[y] = readRawTemperature(TEMP_OBJECT);
			if (abs(lastRawVal - rawValues[y]) > MAX_SAFE_TEMPERATURE_CHANGE) {
				// The temperature changed too much. Give the sensor time to warm up or cool down, and try again.
				delay(MIN_READ_WAIT);
				rawValues[y] = readRawTemperature(TEMP_OBJECT);
			}
			lastRawVal = rawValues[y];
			if (y != lines - 1) {
				yPos += yInc;
				ud.writeMicroseconds(yPos);
				// We need to wait long enough in this new position to read the new temperature.
				delay(MIN_READ_WAIT);
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
	finish();
}


void saveTemperaturePixelToSD(int *rawValues, int length){	
	char st1[32];      
	sdRes=file.initFAT();
	if (file.exists(newFileNameChars)) file.delFile(newFileNameChars);	
	file.create(newFileNameChars);
	sdRes=file.openFile(newFileNameChars, FILEMODE_TEXT_WRITE);
	for (int idx = 0; idx < length; idx++) {
		count++;
		long tpl = (long)(rawValues[idx]) * 2 - TEMP_OFFSET;
		// Since Arduino does not support float printing, we emulate it.
		sprintf(st1, "%s%li.%02li", (tpl < 0 ? "-" : ""), abs(tpl / 100), abs(tpl % 100)); 	
		if (sdRes==NO_ERROR)
		{    
			file.writeLn(st1);			
		}       
	}	
	if (sdRes==NO_ERROR)
	{   		
		file.closeFile();
	}	
}

void finish(){    
	count = 0; // Reset the count.
	ud.writeMicroseconds(mud);
	lr.writeMicroseconds(mlr);
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
		file.delFile("FILES.SYS");	}
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
					renderTemperature = atof((char*)sdTextBuffer)*100;		  
					if(renderMaxT < renderTemperature){ renderMaxT = renderTemperature; }
					if(renderMinT > renderTemperature){ renderMinT = renderTemperature; }           
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
					renderTemperature = atof((char*)sdTextBuffer)*100;		  
					renderDrawPixelTemperature(renderTemperature,x,y);
					y--;
					if(y<0) { x++; y=47; }     
					
				}        
			}      
			file.closeFile();
		}    
	}
	myGLCD.setColor(240,240,240);	
	myGLCD.print(currFileNameChars,CENTER, 228);
}

void saveTMP(){
	
	sdRes=file.initFAT(); 
	if (file.exists(newFileNameChars)) file.delFile(newFileNameChars);	
	file.create(newFileNameChars);
	sdRes=file.openFile(newFileNameChars, FILEMODE_TEXT_WRITE);
	if (sdRes==NO_ERROR)
	{    
		file.writeLn(newFileNameChars);
		file.writeLn("this is new another one");			
		file.closeFile();
	}

}



void setup()
{  		
	Serial.begin(19200);
	myGLCD.InitLCD();
	myGLCD.clrScr();     
	myGLCD.setBackColor(0, 0, 0);	  
	myGLCD.setFont(SmallFont);
	mlr = 1275; //Middle point for LR Servo; only used if there is no existing calibration
	mud = 1750; //Middle point for UD Servo; only used if there is no existing calibration  
	blr = 1600; //Left-Bottom point for LR Servo; only used if there is no existing calibration
	bud = 1550; //Left-Bottom point for UD Servo; only used if there is no existing calibration
	smb_init(true);
	ud.attach(VERT_SERVO_PIN); //Attach servos
	lr.attach(HORIZ_SERVO_PIN); 
	ud.writeMicroseconds(mud); //Move servos to middle position
	lr.writeMicroseconds(mlr);
	
	//main process
	makeNewFilename();
	delay(500);
	scan();
	strcpy(currFileNameChars,newFileNameChars);	
	renderFindMaxMinT();
	renderResult();		
}



void loop()
{ 
	delay(1000);
}

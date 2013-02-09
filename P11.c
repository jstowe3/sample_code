/****** ThermostatAlert.c **********************************************
 *
 * @HANDLE = SWV
 * @AUTHOR = JOSEPH STOWE AND WON HWANG
 * 
 * Interface with the SHT15 sensor using digital 2-wire interface.
 * Poll the SHT15 for temperature and humidity measurements approx every
 * 2 seconds. Also calculates and displays dewpoint based on temp and humidity.
 * Allows user to set min and max values on humidty and triggers alerts
 * during a breach in the form of an on-screen display as well as 
 * a high-pitch alarm speaker controlled with RD0.
 * 
 **********************************************************************/
#include "p24FJ256GB110.h"       // PIC24 register and bit definitions
#include "AlphaFont.h"           // 12x16-pixel font set
#include "Mikro.c"               // LCD variables, functions, macros
#include "MikroTouch.c"          // Touchscreen variables, functions
#include "MikroMeasureTime.c"    // Start, Stop, Send, ASCIIn, Blankn functions
#include "MikroDebug.c"          // Debugging functions
#include "MikroI2C.c"            // I2C functions
#include "math.h"                // Math libraries used for calculating dew point

/****** Configuration selections **************************************/
_CONFIG1(JTAGEN_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx2);
_CONFIG2(PLLDIV_DIV2 & POSCMOD_HS & FNOSC_PRIPLL & IOL1WAY_OFF);

/****** Global variables **********************************************/
char HexStr[] = "\001\0010x0000";
unsigned int ALIVECNT = 0;
unsigned int ALIVECNT2 = 0;
char Potstr[] = "\010\030000";

char XPIXELstr[] = "\006\010000";
char YPIXELstr[] = "\006\020000";

char HANDLEstr[] = "\001\002SWV:";
char TITLEstr[] = "\001\010Humidity and Temp";

char TempDec[] = "\007\00100000";
char TempFhr[] = "\004\001000.00 C";
char HumidityDec[] = "\005\0010000";
char HumidityRel[] = "\006\001000.00%";
char DewPointstr[] = "\003\001000.00 C";

char TargetStr1[] = "\002\003Max Temp: 00  C";
char TargetStr3[] = "\002\003Min Temp: 00  C";
char TargetStr2[] = "\002\003Max Humid: 000%";
char TargetStr4[] = "\002\003Min Humid: 000%";

char CurTempStr[] = "\005\015Temp: 00.00 C";
char CurHumidStr[] = "\006\014Humid: 00.00 %";
char CurDewPointStr[] = "\007\013DewPnt: 00.00 C";

char AlertStr1[] = "\003\011!!!";
char AlertStr2[] = "\004\011!!!";
char AlertStr3[] = "\003\026!!!";
char AlertStr4[] = "\004\026!!!";
char BlankStr1[] = "\003\011   ";
char BlankStr2[] = "\004\011   ";
char BlankStr3[] = "\003\026   ";
char BlankStr4[] = "\004\026   ";

float CurrentTemp;
float CurrentHumidity;

char TargetChange = 0;

char IsConfirmed = 0;           // boolean to tell wether the user pressed the pushbutton

char MaxTemp = 50;
char MinTemp = -10;
char MaxHumid = 100;
char MinHumid = 0;
char MaxTempCpy = 50;
char MaxHumidCpy = 100;
char MinTempCpy = -10;
char MinHumidCpy = 0;


signed char DELRPG = 0;         // Change variable for RPG
unsigned int OLDRPG, NEWRPG;    // Variables to detect a change in the RPG value


/****** Function prototypes *******************************************/
void Initial(void);
void BlinkAlive(void);
void DisplayHandle(void);
void InitRPG(void);
void RPG(void);
int BoundsDetect(int x1, int y1, int x2, int y2);
void ReadHumidity(void);
void ReadTemp(void);
void DewPoint(void);
void InitDisplay(void);
void DetectTarget(void);
void SelectBound(void);
void CheckAlerts(void);

/****** Macros ********************************************************/
#define BLACK RGB(0,0,0)
#define SILVER RGB(192,192,192)
#define GRAY RGB(128,128,128)
#define WHITE RGB(255,255,255)
#define MAROON RGB(128,0,0)
#define RED RGB(255,0,0)
#define PURPLE RGB(128,0,128)
#define FUCHSIA RGB(255,0,255)
#define GREEN RGB(0,128,0)
#define LIME RGB(0,255,0)
#define OLIVE RGB(128,128,0)
#define YELLOW RGB(255,255,0)
#define NAVY RGB(0,0,128)
#define BLUE RGB(0,0,255)
#define TEAL RGB(0,128,128)
#define AQUA RGB(0,255,255)

//////// Main program //////////////////////////////////////////////////

int main()
{
   Initial();
   
   while (1)                     // Looptime without Sleep = 20.35 ms
   {
      
	  DetectTouch();             // Detect current touch on screen
	  DetectTarget();            // Determine if target value has changed
	  RPG();                     // Update DELRPG value
	  SelectBound();             // Change a target value based on DELRPG
	  ReadHumidity();            // Read relative humidity every 2 sec
	  ReadTemp();                // Read temperature every 2 sec
	  CheckAlerts();             // Check to update alerts
      
      while (!_T5IF) ;           // Loop time = 10 ms
      _T5IF = 0;
   }
}

/****** Initial ********************************************************
 *
 * Initialize LCD Screen (PMP + configuration + initial display).
 * Initialize Timer5 for a loop time of 10 ms.
 **********************************************************************/
void Initial()
{
   AD1PCFGL = 0xFFFF;            // Make all ADC pins default to digital pins
   PMP_Init();                   // Configure PMP module for LCD
   LCD_Init();                   // Configure LCD controller
   InitRPG(); // Initialize the RPG
   InitBackground();             // Paint screen royal blue
   DisplayHandle();              // Display handle
   InitDisplay();
   _TRISD0 = 0;                  // Make RD0 an output (pin 50 of Mikro board)
   TMR5 = 0;                     // Clear Timer5
   PR5 = 19999;                  // Set period of Timer5 to 10 ms
   T5CON = 0x8010;               // Clock Timer5 with Fcy/8 = 2 MHz
   
}

/****** InitDisplay ********************************************************
 *
 * Initialize all elements of the display.
 * 
 **********************************************************************/
void InitDisplay()
{
	static char MaxTempLabel[] = "\003\003Max C";
	static char MaxHumidLabel[] = "\003\020Max H";  // Row 13
	static char MinHumidLabel[] = "\004\020Min H";
	static char MinTempLabel[] = "\004\003Min C";
	DrawRectangle(5, 53, 21, 69, YELLOW);
	Display(BKGD, MaxTempLabel);
	
	DrawRectangle(155, 53, 171, 69, YELLOW);
	Display(BKGD, MinTempLabel);
	
	DrawRectangle(5, 77, 21, 93, YELLOW);
	Display(BKGD, MaxHumidLabel);

	DrawRectangle(155, 77, 171, 93, YELLOW);
	Display(BKGD, MinHumidLabel);

	Display(BKGD, TargetStr1);

	Display(BKGD, CurTempStr);
	Display(BKGD, CurHumidStr);
	Display(BKGD, CurDewPointStr);
}

/****** CheckAlerts ********************************************************
 *
 * Check to see if boundaries have been breeched by measurements
 * 
 **********************************************************************/
void CheckAlerts()
{
	static char Alert1Fixed = 0;
	static char Alert2Fixed = 0;
	static char Alert3Fixed = 0;
	static char Alert4Fixed = 0;
	
	if(CurrentTemp < (float)MinTemp && Alert2Fixed == 0)
	{
		Alert2Fixed = 1;
		Display(RED, AlertStr2);
	}
	else if(CurrentTemp >= (float)MinTemp && Alert2Fixed ==1)
	{
		Alert2Fixed = 0;
		Display(BKGD, BlankStr2);
	}

	if(((CurrentTemp - MaxTemp) > 0) && (Alert1Fixed == 0))
	{
		Alert1Fixed = 1;
		Display(RED, AlertStr1);
	}
	else if(CurrentTemp <= (MaxTemp*1.00) && Alert1Fixed ==1)
	{
		Alert1Fixed = 0;
		Display(BKGD, BlankStr1);
	}

	if(CurrentHumidity > (float)MaxHumid && Alert3Fixed == 0)
	{
		Alert3Fixed = 1;
		Display(RED, AlertStr3);
	}
	else if(CurrentHumidity <= (float)MaxHumid && Alert3Fixed ==1)
	{
		Alert3Fixed = 0;
		Display(BKGD, BlankStr3);
	}

	if(CurrentHumidity < (float)MinHumid && Alert4Fixed == 0)
	{
		Alert4Fixed = 1;
		Display(RED, AlertStr4);
	}
	else if(CurrentHumidity >= (float)MinHumid && Alert4Fixed ==1)
	{
		Alert4Fixed = 0;
		Display(BKGD, BlankStr4);
	}

   if(Alert1Fixed || Alert2Fixed || Alert3Fixed || Alert4Fixed)
   {
      _LATD0 = 1; // Sound alarm speaker
   }
   else
   {
      _LATD0 = 0; // Turn speaker off
   }
	
}

/****** SelectBound ********************************************************
 *
 * Modify the bounds of a target based on change in DELRPG
 * 
 **********************************************************************/
void SelectBound()
{
	int temp_value;
	char make_change = 0;
	

	IsConfirmed = !_RB0;      // Check wether pushbutton is pressed
	if(IsConfirmed)
	{
		IsConfirmed = 0;      // Reset value to zero
		make_change = 1;
	}
	
	if(TargetChange == 0)     // Modify Max Temp
	{
		if(make_change)
		{
			make_change = 0;
			MaxTemp = MaxTempCpy;
		}

		if(DELRPG ==1 && MaxTempCpy < 50)
		{
			
			MaxTempCpy += 1;  // Increment MaxTemp value
		}
		else if(DELRPG == -1 && MaxTempCpy > -10)
		{
			MaxTempCpy -= 1;  // Decrement MaxTemp
		}
		temp_value = MaxTempCpy;
		if(MaxTempCpy < 0)
		{
			TargetStr1[11] = '-';
			temp_value = temp_value * -1;
		}
		else
		{
			TargetStr1[11] = ' ';
		}
		TargetStr1[12] ='0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr1[13] ='0' + temp_value;
		Display(BKGD, TargetStr1);
	}
	else if(TargetChange == 1)    // Modify Max Humidity
	{

		if(make_change)
		{
			make_change = 0;
			MaxHumid = MaxHumidCpy;
		}

		if(DELRPG ==1 && MaxHumidCpy < 100)
		{
			
			MaxHumidCpy += 1;    // Increment MaxHumid value
		}
		else if(DELRPG == -1 && MaxHumidCpy > 0)
		{
			MaxHumidCpy -= 1;    // Decrement MaxTemp
		}
		temp_value = MaxHumidCpy;
		
		TargetStr2[13] ='0' + temp_value/100;
		temp_value = temp_value % 100;
		TargetStr2[14] = '0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr2[15] ='0' + temp_value;		

		Display(BKGD, TargetStr2);
	}
	else if(TargetChange == 2)   // Modifty Min Temp
	{

		if(make_change)
		{
			make_change = 0;
			MinTemp = MinTempCpy;
		}

		if(DELRPG ==1 && MinTempCpy < 50)
		{
			
			MinTempCpy += 1;    // Increment MinTemp value
		}
		else if(DELRPG == -1 && MinTempCpy > -10)
		{
			MinTempCpy -= 1;    // Decrement MaxTemp
		}
		temp_value = MinTempCpy;
		if(MinTempCpy < 0)
		{
			TargetStr3[11] = '-';
			temp_value = temp_value * -1;
		}
		else
		{
			TargetStr3[11] = ' ';
		}
		TargetStr3[12] ='0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr3[13] ='0' + temp_value;

		Display(BKGD, TargetStr3);

	}
	else  // Modify Min Humidity
	{

		if(make_change)
		{
			make_change = 0;
			MinHumid = MinHumidCpy;
		}

		if(DELRPG ==1 && MinHumidCpy < 100)
		{
			
			MinHumidCpy += 1;    // Increment MaxHumid value
		}
		else if(DELRPG == -1 && MinHumidCpy > 0)
		{
			MinHumidCpy -= 1;    // Decrement MaxTemp
		}
		temp_value = MinHumidCpy;
		
		TargetStr4[13] ='0' + temp_value/100;
		temp_value = temp_value % 100;
		TargetStr4[14] = '0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr4[15] ='0' + temp_value;		

		Display(BKGD, TargetStr4);

	}
	

}


/****** ReadHumidity ********************************************************
 *
 * Read current humidity value from the SHT15 sensor
 * 
 **********************************************************************/
void ReadHumidity()
{

	static int HUMID_COUNT = 0;

	static int prevRead = 0;

   int response;
   int temp_response;
   char printPermit = 0;
   float floatVal;
   float c1 = -2.0468;
   float c2 = 0.0367;
   float c3 = -0.0000015955;
   

	HUMID_COUNT++;
	if (HUMID_COUNT == 100)
	{
		HUMID_COUNT = 0;
		
		sht15_start();  // Send start command to sensor
	
   	sht15_command(0b00000101);
	
   	response = sht15_read_byte16();
      
      temp_response = response;                   // Make a copy of the humidity #

	  //DisplayInt(6, response);
      //HumidityDec[2] = '0' + (response /1000);  // Thousands place 
      //response = response%1000;
      //HumidityDec[3] = '0' + (response / 100);  // Hundreds place
      //response = response%100;
      //HumidityDec[4] = '0'+ (response / 10);    // Tens place
      //response = response%10;
      //HumidityDec[5] = '0'+ (response);         // Ones place
      //Display(BKGD, HumidityDec);

	  // Calculate relative humidity:
      floatVal = c1 + (c2*temp_response) + (c3*(temp_response*temp_response));

      CurrentHumidity = floatVal;                // Save humidity to global variable

	  temp_response = (int)floatVal;             // Truncate float value to integer

	  if(temp_response != prevRead)
	  {
		  prevRead = temp_response;
		  printPermit = 1;  // allow redraw for bar graph
	  }

	  floatVal = floatVal - temp_response;       // We are left with the number after the decimal

      CurHumidStr[8] = '0' + (temp_response/100);
      temp_response = temp_response%100;
      CurHumidStr[9] = '0' + (temp_response/10);
      temp_response = temp_response%10;
      CurHumidStr[10] = '0' + temp_response/1;
	  CurHumidStr[12] = '0' + ((int)(floatVal*10)); // 10th's place after decimal
	  temp_response = (int)(floatVal*100);      // Multiply by 100 and convert to integer
	  temp_response = temp_response % 10;       // Get the 100th's place
	  CurHumidStr[13] = '0' + temp_response;    // Hundredth's place after decimal
      Display(BKGD, CurHumidStr);
	  
	  temp_response = ((int)CurrentHumidity);   //Cast temp to integer
	  
	  if(printPermit)
	  {
	  	DrawRectangle(5,125,5+130,141,BKGD);    // Clear graph
	  	DrawRectangle(5,125,5+(temp_response),141, LIME);
	  }
	  
      DewPoint();                              // Calculate Dew Point
	}

}

/****** ReadTemp ********************************************************
 *
 * Read current temperature value from the SHT15 sensor
 * 
 **********************************************************************/
void ReadTemp()
{

	static int prevRead = 0;

	static int TEMP_COUNT = 50;
   int response;
   int temp_response;
   float floatVal;
   float d1 = -39.7;
   float d2 = 0.01;
	char printPermit = 0;
   

	TEMP_COUNT++;
	if (TEMP_COUNT == 100)
	{
		TEMP_COUNT = 0;
		
		sht15_start();                          // Send start command to sensor
	
   	sht15_command(0b00000011);
	
   	response = sht15_read_byte16();
   
      temp_response = response;                // Save a copy of temperature
	  //DisplayInt(8, response);
      //TempDec[2] = '0' + (response /10000);
      //response = response%10000;
      //TempDec[3] = '0' + (response /1000);   // Thousands place 
      //response = response%1000;
      //TempDec[4] = '0' + (response / 100);   // Hundreds place
      //response = response%100;
      //TempDec[5] = '0'+ (response / 10);     // Tens place
      //response = response%10;
      //TempDec[6] = '0'+ (response);          // Ones place
      //Display(BKGD, TempDec);

      floatVal = d1 + (d2*temp_response);

	  CurrentTemp = floatVal;                  // Save temp to global variable

	  temp_response = (int)floatVal;           // Convert to int value and store in temp_val

	  if(prevRead != temp_response)
	  {
		  prevRead = temp_response;
		  printPermit = 1;
	  }

	  floatVal = floatVal - temp_response;    // Get value after the decimal;

      CurTempStr[8] = '0' + temp_response/10;
      temp_response = temp_response%10;
      CurTempStr[9] = '0' + temp_response/1;
	  CurTempStr[11] = '0' + ((int)(floatVal*10)); // 10th's place after decimal
	  temp_response = (int)(floatVal*100);   // Multiply by 100 and convert to integer
	  temp_response = temp_response % 10;    // Get the 100th's place
	  CurTempStr[12] = '0' + temp_response;  // Hundredth's place after decimal
      Display(BKGD, CurTempStr);

	  temp_response = ((int)CurrentTemp)+10; //Cast temp to integer and add 10
		
	  if(printPermit)
	  {
	  	DrawRectangle(5,101,5+130,117,BKGD); // Clear graph
	  	DrawRectangle(5,101,5+(2*temp_response),117, LIME);
	  }
	}

}

/****** DewPoint ********************************************************
 *
 * Calculate approximate dew point based on current temp and humidity
 * Then display the current dew point on the LCD
 *
 **********************************************************************/
void DewPoint()
{

	static char prevRead = 0;
   float Tn = 243.12;
   float m = 17.62;
   float RH = CurrentHumidity;
   float T = CurrentTemp;
   float Dewpoint;
   int DewPntCpy;
   int intVal;
	char printPermit = 0;

   Dewpoint = Tn*((log(RH/100)+((m*T)/(Tn+T)))/(m-log(RH/100)-((m*T)/(Tn+T)))); // Calculate current dewpoint
	
   DewPntCpy = ((int)Dewpoint) + 10;     // Save int value of dewpoint + 10;
   
   intVal = (int)Dewpoint;               // Cast to integer and store result in inVal
	
	if(intVal != prevRead)
	{
		prevRead = intVal;
	    printPermit = 1;
	}

   Dewpoint = Dewpoint - intVal;         // Get number after the decimal place

   CurDewPointStr[10] = '0' + intVal/10;
   intVal = intVal%10;
   CurDewPointStr[11] = '0' + intVal/1;
   CurDewPointStr[13] = '0' + ((int)(Dewpoint*10)); // 10th's place after decimal
   intVal = (int)(Dewpoint*100);        // Multiply by 100 and convert to integer
   intVal = intVal % 10;                // Get the 100th's place
   CurDewPointStr[14] = '0' + intVal;   // Hundredth's place after decimal
   Display(BKGD, CurDewPointStr);

   if(printPermit)
	{
   		DrawRectangle(5,149,5+115,165,BKGD); // Clear graph
   		DrawRectangle(5,149,5+(2*DewPntCpy),165, LIME);
	}
}


/****** InitRPG ********************************************************
 *
 * Initialize the RPG by enabling internal pullups on RB0,RB2,RB3.
 *
 **********************************************************************/
void InitRPG()
{
   _CN2PUE = 1;                  // Enable pullup on RB0/CN2 for pushbutton
   _CN4PUE = 1;                  // Enable pullup on RB2/CN4 for RPGx
   _CN5PUE = 1;                  // Enalbe pullup on RB3/CN5 for RPGy
   Nop();                        // Pause for one cycle (i.e., one microsecond)
   OLDRPG = (PORTB & 0x000C);    // Form initial value of OLDRPG
}

/***********************************************************************
 * RPG
 *
 * This function checks the rotary pulse generator for a change.
 * DELRPG = 0 for no change; +1 for a CW change; -1 for a CCW change.
 **********************************************************************/
void RPG()
{
   DELRPG = 0;                   // Reset DELRPG to a default output value of zero
   NEWRPG = (PORTB & 0x000C);    // Read in NEWRPG

   if (NEWRPG != OLDRPG)         // A change has occurred
   {
      if (0x0008 & (OLDRPG ^ (NEWRPG << 1)))    // Counter-clockwise
      {
         DELRPG = +1;
      }
      else                       // Clockwise
      {
         DELRPG = -1;
      }
      OLDRPG = NEWRPG;           // Save changed RPG value for next time
   }
}


/****** DetectTarget ********************************************************
 *
 * Update the current Target variable control
 *
 **********************************************************************/

void DetectTarget()
{

   int temp_value;

   if (BoundsDetect(5, 53, 40, 69))
   {
      TargetChange = 0;          // Save current target value
	
	  temp_value = MaxTemp;
		if(MaxTemp < 0)
		{
			TargetStr1[11] = '-';
			temp_value = temp_value * -1;
		}
		else
		{
			TargetStr1[11] = ' ';
		}
		TargetStr1[12] ='0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr1[13] ='0' + temp_value;

	  MaxTempCpy = MaxTemp;      // Reset the copy
	  
	  Display(BKGD, TargetStr1); // Update current target string
   }
   if (BoundsDetect(155, 53, 190, 69))
   {
      TargetChange = 1;          // Save current target value

	  temp_value = MaxHumid;
		
		TargetStr2[13] ='0' + temp_value/100;
		temp_value = temp_value % 100;
		TargetStr2[14] = '0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr2[15] ='0' + temp_value;

	  MaxHumidCpy = MaxHumid;    // Reset the copy

	  Display(BKGD, TargetStr2); // Update current target string
   }
   if (BoundsDetect(5, 77, 40, 93))
   {
      TargetChange = 2;          // Save current target value

	  temp_value = MinTemp;
		if(MinTemp < 0)
		{
			TargetStr3[11] = '-';
			temp_value = temp_value * -1;
		}
		else
		{
			TargetStr3[11] = ' ';
		}
		TargetStr3[12] ='0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr3[13] ='0' + temp_value;

	  MinTempCpy = MinTemp;       // Reset the copy

	  Display(BKGD, TargetStr3);  // Update current target string
   }
   if (BoundsDetect(155, 77, 190, 93))
   {
      TargetChange = 3;           // Save current target value

	  temp_value = MinHumid;
		
		TargetStr4[13] ='0' + temp_value/100;
		temp_value = temp_value % 100;
		TargetStr4[14] = '0' + temp_value/10;
		temp_value = temp_value % 10;
		TargetStr4[15] ='0' + temp_value;

	  MinHumidCpy = MinHumid;     // Reset the copy

	  Display(BKGD, TargetStr4);  // Update current target string
   }
}

/****** BoundsDetect ********************************************************
 *
 * Detect whether a block has been touched
 *
 **********************************************************************/
int BoundsDetect(int x1, int y1, int x2, int y2)
{
   unsigned char result = 0;

   if ((tsx >= x1) && (tsx <= x2))
   {
      if ((tsy >= y1) && (tsy <= y2))
      {
         result = 1;              // Touch detected in our box
      }
   }
   return result;
}


/****** DisplayHandle ********************************************************
 *
 * Display the Handle centered on the first line
 *
 **********************************************************************/
void DisplayHandle()
{
   Display(BKGD, HANDLEstr);     // Display the handle string

   Display(BKGD, TITLEstr);      // Display the title string below
}


/****** BlinkAlive *****************************************************
 *
 * This function toggles a square of pixels every second.
 * With a loop time of 10 ms, this is every 100 looptimes.
 **********************************************************************/
void BlinkAlive()
{
   ALIVECNT++;
   if (ALIVECNT == 100)          // Write black square
   {
      DrawRectangle(0, 0, 5, 5, BLACK);
   }
   if (ALIVECNT >= 200)          // Clear black square
   {
      ALIVECNT = 0;
      DrawRectangle(0, 0, 5, 5, BKGD);
   }
}
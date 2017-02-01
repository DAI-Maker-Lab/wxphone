#define VERSION "0.2.0"      //  Version number

//  Customize these items for each installation
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#define PUBLIC_KEY "dZ9j1JqgEwtqp8QGyQLn"      //  Public key for phant stream
#define PRIVATE_KEY "eEJjA1V5XxTD5mlMxl0B"    //  Private key for phant stream
#define INITIAL_LAT 39.020000
#define INITIAL_LON -77.050000
#define INITIAL_ALT 122

#define UTCOFFSET -5        //  Local standard time variance from UTC
#define XBEEWINDOWSTART 8   //  Hour to turn on XBee for programming window
#define XBEEWINDOWEND 20    //  Hour to turn off XBee
#define INTERVAL 15         //  Number of minutes between readings
#define BEEPASSWORD "XBEE_PASSWORD"  //  Password to turn on XBee by SMS
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



#include <Adafruit_BME280.h>    //  Pressure/temp/humidity sensor
#include <Adafruit_FONA.h>      //  For Adafruit FONA module
#include <Adafruit_Sensor.h>    //  Adafruit unified sensor abstraction layer
#include <DS1337.h>             //  For the Stalker's real-time clock (RTC)
#include <ss_no_int3.h>         //  SoftwareSerial with pin-change conflict removed
#include <Sleep_n0m1.h>         //  Sleep library
#include <SPI.h>
#include <Wire.h>               //  I2C needed for sensors



#define RTCINT 0                //  RTC interrupt number
#define WDIR A0                 //  Wind direction from vane
#define FONA_RX A1              //  FONA RX pin
#define FONA_RST A2             //  FONA reset pin
#define WSPEED 3                //  Wind speed interrupt pin
#define FONA_TX 7               //  FONA TX pin
#define BEEPIN 5                //  XBee power pin
#define FONA_KEY 6              //  FONA on/off pin
#define FONA_PS 4               //  FONA power status pin
#define RAIN 13                 //  Rain gauge interrupt pin

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile float rainMinute[60];      //  60 floating numbers to keep track of 60 minutes of rain

//  Weather values
char* wind_dir;                    //  Instantaneous wind direction
float wind_speed = 0;              //  km/h instantaneous wind speed
float temp = 0;                    //  temperature C
float humidity = 0;                //  %
float pressure = 0;
float rainHourly[24];              //  24 floats to track the last 24 hours of rain
float rain_today = 0.00;           //  Rain since midnight (mm)

volatile unsigned long raintime, rainlast, raininterval, rain;

boolean dataSent = false;

byte beeShutoffHour = 0;        //  Hour to turn off manual power to XBee
byte beeShutoffMinute = 0;      //  Minute to turn off manual power to XBee
boolean smsPower = false;       //  Manual XBee power flag

/* We will create variables to hold  values for the time and date from the RTC. */
int currentHour;    
byte currentMinute;     
byte currentDay;
byte currentMonth;
int currentYear;
DateTime now;

float latitude = INITIAL_LAT;
float longitude = INITIAL_LON;
float altitude = INITIAL_ALT;

ss_no_int3 fonaSerial = ss_no_int3 (FONA_TX, FONA_RX);
Adafruit_FONA fona = Adafruit_FONA (FONA_RST);

Adafruit_BME280 bme;              //  Instantiate BME280 sensor object
DS1337 RTC;                       //  Instantiate real-time clock (RTC) object
Sleep sleep;                      //  Instantiate sleep object



void pciSetup()
{
        //  Setup pin change interrupt for rain pin
        *digitalPinToPCMSK(RAIN) |= bit (digitalPinToPCMSKbit(RAIN));  //   Enable pin
        PCIFR  |= bit (digitalPinToPCICRbit(RAIN)); //    Clear any outstanding interrupt
        PCICR  |= bit (digitalPinToPCICRbit(RAIN)); //    Enable interrupt for the group
}



static void rtcIRQ()
{
        RTC.clearINTStatus();   //  Wake from sleep and clear the RTC interrupt
}



ISR (PCINT0_vect)                    //  Count rain gauge bucket tips as they occur
{
        raintime = millis();       
        raininterval = raintime - rainlast;  //  calculate interval between this and last event
      
        if (raininterval > 40)  //  ignore switch-bounce glitches less than 10mS after initial edge
        {
                rain_today += 0.2794;                //  Each dump is 0.2794mm of water
                rainMinute[currentMinute] += 0.2794; //  Increase this minute's amount of rain
                rainlast = raintime;                 //  Set up for next event
        }
}



static void wspeedIRQ()      //  Activated by the magnet in the anemometer (2 ticks per rotation)
{
        if (millis() - lastWindIRQ > 10)  //  Ignore switch-bounce glitches less than 10ms
        {
                lastWindIRQ = millis();   
                windClicks++;             //  Each click per second represents 2.40264 km/h of wind
        }
}



void setup()
{
        Wire.begin(); //  Begin the I2C interface
        RTC.begin();  //  Begin the RTC        
        Serial.begin (57600);
        Serial.print (F("wxPhone version "));
        Serial.print (VERSION);
        Serial.print (F(" "));
        Serial.print (__DATE__);    //  Compile data and time helps identify software uploads
        Serial.print (F(" "));
        Serial.println (__TIME__);

        analogReference (INTERNAL); 

        Serial.print (F("Startup voltage "));
        Serial.println (analogRead (A7) * 6.4);   //  Convert ADC reading to voltage
        
        pinMode (BEEPIN, OUTPUT);
        pinMode (FONA_KEY, OUTPUT);
        pinMode (FONA_PS, INPUT);       
        pinMode (FONA_RX, OUTPUT);
        pinMode (WSPEED, INPUT_PULLUP);
        pinMode (RAIN, INPUT_PULLUP);
        pinMode (WDIR, INPUT);


        /*   If the voltage at startup is less than 3.5V, we assume the battery died in the field
         *   and the unit is attempting to restart after the panel charged the battery enough to
         *   do so. However, running the unit with a low charge is likely to just discharge the
         *   battery again, and we will never get enough charge to resume operation. So while the
         *   measured voltage is less than 3.5V, we will put the unit to sleep and wake once per 
         *   hour to check the charge status.
         */

        while (analogRead (A7) < 547)   
        {
                Serial.print (F("Battery voltage is only "));
                Serial.print (analogRead (A7) * 6.4);
                Serial.print (F("mV. Sleeping to save power while panel recharges..."));
                Serial.flush();
                digitalWrite (BEEPIN, HIGH);      //  Make sure XBee is powered off
                RTC.enableInterrupts (EveryHour); //  We'll wake up once an hour
                RTC.clearINTStatus();             //  Clear any outstanding interrupts
                attachInterrupt (RTCINT, rtcIRQ, FALLING);
                interrupts();
                sleep.pwrDownMode();                    //  Set sleep mode to Power Down
                sleep.sleepInterrupt (RTCINT, FALLING); //  Sleep; wake on falling voltage on RTC pin
        }

        digitalWrite (FONA_KEY, HIGH);       //  Initial state for key pin

        //  We will use the FONA to get the current time to set the Stalker's RTC
        fonaOn();
        clockSet();
        
        //  Delete any accumulated SMS messages to avoid interference from old commands
        fona.sendCheckReply (F("AT+CMGF=1"), F("OK"));            //  Enter text mode
        fona.sendCheckReply (F("AT+CMGDA=\"DEL ALL\""), F("OK")); //  Delete all SMS messages

        gpsFix();
        
        fonaOff();

        pciSetup();
        RTC.enableInterrupts (EveryMinute);  //  RTC will interrupt every minute
        RTC.clearINTStatus();                //  Clear any outstanding interrupts
        attachInterrupt (RTCINT, rtcIRQ, FALLING);

        now = RTC.now();    //  Get the current time from the RTC
        
        //  We'll keep the XBee on for an hour after startup to assist installation
        if (now.hour() == 23)
        {
                beeShutoffHour = 0;
        }
        else
        {
                beeShutoffHour = (now.hour() + 1);
        }
        beeShutoffMinute = now.minute();

        Serial.print (F("XBee powered on until at least "));
        Serial.print (beeShutoffHour);
        Serial.print (F(":"));
        Serial.println (beeShutoffMinute);
        Serial.flush();
        smsPower = true; 
        
        if (bme.begin() == false) 
        {
                Serial.println(F("Could not find a valid BME280 sensor; check wiring."));
                while (1);
        }
        else
        {
                Serial.print (F("Humidity: "));
                Serial.println (bme.readHumidity());
                Serial.print (F("Local pressure: "));
                Serial.println (bme.readPressure() / 100);
                Serial.print (F("Sea level pressure: "));
                getPressure();
                Serial.println (pressure);
                Serial.print (F("Temperature: "));
                Serial.println (bme.readTemperature());
        }

        analogReference (DEFAULT);
      
        // attach external interrupt pins to IRQ functions
        //attachInterrupt(0, rainIRQ, FALLING);
        attachInterrupt(1, wspeedIRQ, FALLING);
      
        interrupts();      
}



void loop()
{
        now = RTC.now();   //  Get the current date/time
        
        if(currentMinute != now.minute())
        {
                currentMinute = now.minute(); 
                rainMinute[currentMinute] = 0; 
                
                if(currentMinute == 0)    //  Every hour, update the hourly rain count
                {
                        int lastHour;
                        currentHour = now.hour();
                        
                        if(currentHour == 0)
                        {
                                lastHour = 23;
                        }
                        else
                        {
                                lastHour = currentHour - 1;
                        }
                        
                        rainHourly[lastHour] = 0;
                        
                        for(int i = 0 ; i < 60 ; i++)
                        {
                                rainHourly[lastHour] += rainMinute[i];
                        }
                }
        }

        //  We will turn on the XBee radio for programming only within a specific
        //  window to save power
        if (now.hour() >= XBEEWINDOWSTART && now.hour() <= XBEEWINDOWEND)
        {
                digitalWrite (BEEPIN, LOW);
        }
        else
        {
                //  If the XBee power was turned on by SMS, we'll check to see if 
                //  it's time to turn it back off
                if(smsPower == true && now.hour() < beeShutoffHour)
                {
                        digitalWrite (BEEPIN, LOW);
                }
                else
                {
                        if(smsPower == true && now.hour() == beeShutoffHour && now.minute() < beeShutoffMinute)
                        {
                                digitalWrite (BEEPIN, LOW);
                        }
                        else
                        {
                                if (smsPower == true)
                                {
                                          Serial.println (F("Turning XBee off..."));
                                          Serial.flush();
                                          wait (500);
                                }

                                digitalWrite (BEEPIN, HIGH);
                                smsPower = false;
                        }
                }
        }

        //  If the sent flag is up and this is not an upload minute, lower it
        if(currentMinute % INTERVAL != 0 && dataSent == true)  
        {
                dataSent = false;
        }
        
        //  Check to see if upload is necessary; if so, take readings and send them
        if(currentMinute % INTERVAL == 0 && dataSent == false)
        {       
                Serial.println(F("Upload minute; will boot FONA."));
  
                currentHour = now.hour();      
                currentDay = now.date();       
                currentMonth = now.month();
                currentYear = now.year();
        
                calcWeather();  //  Poll the various sensors
        
                fonaOn();     //  Start the Fona
        
                //  We will attempt to upload data to the cloud. If the attempt fails,
                //  we will restart the Fona and try again for up to three attempts.
            
                byte sendAttempts = 0;    //  Reset the counter
            
                while(dataSent == false && sendAttempts <= 2)
                {
                        sendAttempts++;   //  Increment attempt counter
                  
                        if(sendWeather() == false)  //  If upload appears to fail...
                        {
                                Serial.println(F("Upload failed. Restarting Fona."));
                                fonaOff();   //  ...shut the FONA off.
                                delay(5000);      //  Give the FONA a moment
                                fonaOn();         //  Reboot.
                                dataSent = false; //  Ensure flag is down
                        }
                        else
                        {
                                dataSent = true;
                        }
                }
        
                if(currentHour == 0 && currentMinute == 0)  //  At midnight, after upload...
                {
                        rain_today = 0.00;       // ...zero out the day's rain total...
                        clockSet();              // ...and set the time again -- the RTC loses about
                                                 // one minute every six days.
                        if (latitude == INITIAL_LAT)  //  If we never successfully got a GPS fix...
                        {
                                gpsFix();             //  ...try again.
                        }
                }

                fonaOff();           //  Shut down the FONA to save power
        }

        sleep.pwrDownMode();                    //  Set sleep mode to Power Down
        sleep.sleepInterrupt (RTCINT, FALLING); //  Sleep; wake on falling voltage on RTC pin
}



void wait (unsigned long period)
{
        //  Non-blocking delay function
        unsigned long waitend = millis() + period;
        while (millis() <= waitend)
        {
                Serial.flush();
        }
}



boolean fonaOn()
{
        if (digitalRead (FONA_PS) == LOW)             //  If the FONA is off...
        {
                Serial.print (F("Powering FONA on..."));
                while (digitalRead (FONA_PS) == LOW) 
                {
                        digitalWrite(FONA_KEY, LOW);  //  ...pulse the Key pin low...
                        wait (500);
                }
                digitalWrite (FONA_KEY, HIGH);        //  ...and then return it to high
                Serial.println(F(" done."));
        }
        
        Serial.println (F("Initializing FONA..."));
        
        fonaSerial.begin (4800);                      //  Open a serial interface to FONA
        
        if (fona.begin (fonaSerial) == false)         //  Start the FONA on serial interface
        {
                Serial.println (F("FONA not found. Check wiring and power."));
                return false;
        }
        else
        {
                Serial.print (F("FONA online. "));
                
                unsigned long gsmTimeout = millis() + 30000;
                boolean gsmTimedOut = false;

                Serial.print (F("Waiting for GSM network... "));
                while (1)
                {
                        byte network_status = fona.getNetworkStatus();
                        if(network_status == 1 || network_status == 5) break;
                        
                        if(millis() >= gsmTimeout)
                        {
                                gsmTimedOut = true;
                                break;
                        }
                        
                        wait (250);
                }

                if(gsmTimedOut == true)
                {
                        Serial.println (F("timed out. Check SIM card, antenna, and signal."));
                        return false;
                }
                else
                {
                        Serial.println(F("done."));
                }

                //  RSSI is a measure of signal strength -- higher is better; less than 10 is worrying
                byte rssi = fona.getRSSI();
                Serial.print (F("RSSI: "));
                Serial.println (rssi);

                wait (3000);    //  Give the network a moment

                //fona.setGPRSNetworkSettings (F("cellcard"));    //  Set APN to your local carrier

                if (rssi > 5)
                {
                        if (fona.enableGPRS (true) == false);
                        {
                              //  Sometimes enableGPRS() returns false even though it succeeded
                              if (fona.GPRSstate() != 1)
                              {
                                      for (byte GPRSattempts = 0; GPRSattempts < 5; GPRSattempts++)
                                      {
                                            Serial.println (F("Trying again..."));
                                            wait (2000);
                                            fona.enableGPRS (true);
                                            
                                            if (fona.GPRSstate() == 1)
                                            {
                                                    Serial.println (F("GPRS is on."));
                                                    break;
                                            }
                                            else
                                            {
                                                    Serial.print (F("Failed to turn GPRS on... "));
                                            }
                                      }
                              }
                        }
                }
                else
                {
                        Serial.println (F("Can't transmit, network signal strength is poor."));
                        gsmTimedOut = true;
                }
                
                return true;
        }
}



void clockSet()
{
        wait (1000);    //  Give time for any trailing data to come in from FONA

        int netOffset;

        char theDate[17];

        Serial.println (F("Attempting to get time from GSM location service..."));

        flushFona();    //  Flush any trailing data

        fona.sendCheckReply (F("AT+CIPGSMLOC=2,1"), F("OK"));    //  Query GSM location service for time
        
        fona.parseInt();                    //  Ignore first int
        int secondInt = fona.parseInt();    //  Ignore second int -- necessary on some networks/towers
        int netYear = fona.parseInt();      //  Get the results -- GSMLOC year is 4-digit
        int netMonth = fona.parseInt();
        int netDay = fona.parseInt();
        int netHour = fona.parseInt();      //  GSMLOC is _supposed_ to get UTC; we will adjust
        int netMinute = fona.parseInt();
        int netSecond = fona.parseInt();    //  Our seconds may lag slightly, of course

        if (netYear < 2016 || netYear > 2050 || netHour > 23) //  If that obviously didn't work...
        {
                netSecond = netMinute;  //  ...shift everything up one to capture that second int
                netMinute = netHour;
                netHour = netDay;
                netDay = netMonth;
                netMonth = netYear;
                netYear = secondInt;
                
                Serial.println (F("Recombobulating..."));
        }

        if (netYear < 2016 || netYear > 2050 || netHour > 23)   // If that still didn't work...
        {
                Serial.println (F("GSM location service failed."));
                /*   ...the we'll get time from the NTP pool instead: 
                 *   (https://en.wikipedia.org/wiki/Network_Time_Protocol)
                 */
                fona.enableNTPTimeSync (true, F("0.daimakerlab.pool.ntp.org"));
                Serial.println (F("Attempting to enable NTP sync."));
                
                wait (15000);                 // Wait for NTP server response
                
                fona.println (F("AT+CCLK?")); // Query FONA's clock for resulting NTP time              
                netYear = fona.parseInt();    // Capture the results
                netMonth = fona.parseInt();
                netDay = fona.parseInt();
                netHour = fona.parseInt();    // We asked NTP for UTC and will adjust below
                netMinute = fona.parseInt();
                netSecond = fona.parseInt();  // Our seconds may lag slightly
        }

        if ((netYear < 1000 && netYear >= 16 && netYear < 50) || (netYear > 1000 && netYear >= 2016 && netYear < 2050))
        //  If we got something that looks like a valid date...
        {
                //  Adjust UTC to local time
                if((netHour + UTCOFFSET) < 0)                   //  If our offset + the UTC hour < 0...
                {
                        netHour = (24 + netHour + UTCOFFSET);   //  ...add 24...
                        netDay = (netDay - 1);                  //  ...and adjust the date to UTC - 1
                }
                else
                {
                        if((netHour + UTCOFFSET) > 23)          //  If our offset + the UTC hour > 23...
                        {
                                netHour = (netHour + UTCOFFSET - 24); //  ...subtract 24...
                                netDay = (netDay + 1);                //  ...and adjust the date to UTC + 1
                        }
                        else
                        {
                                netHour = (netHour + UTCOFFSET);      //  Otherwise it's straight addition
                        }
                }

                Serial.print (F("Obtained current time: "));
                sprintf (theDate, "%d/%d/%d %d:%d", netDay, netMonth, netYear, netHour, netMinute);
                Serial.println (theDate);
                
                Serial.println(F("Adjusting RTC."));
                DateTime dt(netYear, netMonth, netDay, netHour, netMinute, netSecond, 0);
                RTC.adjust(dt);     //  Adjust date-time as defined above
        }
        else
        {
                Serial.println (F("Didn't find reliable time. Will continue to use RTC's current time."));
        }

        wait (200);              //  Give FONA a moment to catch its breath
}



void flushFona()
{
        // Read all available serial input from FONA to flush any pending data.
        while(fona.available())
        {
                char c = fona.read();
                Serial.print (c);
        }
}



void fonaOff()
{
        wait (5000);        //  Shorter delays yield unpredictable results

        //  We'll turn GPRS off first, just to have an orderly shutdown
        if (fona.enableGPRS (false) == false)
        {
                if (fona.GPRSstate() == 1)
                {
                        Serial.println (F("Failed to turn GPRS off."));
                }
                else
                {
                        Serial.println (F("GPRS is off."));
                }
        }
      
        wait (500);
      
        // Power down the FONA if it needs it
        if (digitalRead (FONA_PS) == HIGH)      //  If the FONA is on...
        {
                fona.sendCheckReply (F("AT+CPOWD=1"), F("OK")); //  ...send shutdown command...
                digitalWrite (FONA_KEY, HIGH);                  //  ...and set Key high
        }
}


void calcWeather()
{
        Wire.begin();
        bme.begin();

        //  Get the instantaneous wind speed over a 5 second sample
        get_wind_speed();
        wait (5000);
        get_wind_speed();
        
        humidity = bme.readHumidity();
        temp = bme.readTemperature();
        getPressure();
 
        wind_dir = get_wind_direction();
        Wire.end();   //  the interface so it starts clean next time.
}



void get_wind_speed()
{
        float deltaTime = millis() - lastWindCheck;
        deltaTime /= 1000.0;       //  Convert to seconds
        wind_speed = (float) windClicks / deltaTime; 
        windClicks = 0;            //  Reset and start watching for new wind
        lastWindCheck = millis();
        wind_speed *= 2.40264;       //  1 click = 2.40264 km/h of windspeed
}



int get_wind_direction()      //  Read the wind direction sensor, return heading
{
        unsigned int adc;
      
        adc = analogRead (WDIR);       //  Get the current reading from the sensor
      
        // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
        // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
        // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
              
        if (adc < 382) return "WNW";
        if (adc < 395) return "WSW";
        if (adc < 415) return "W";
        if (adc < 457) return "NNW";
        if (adc < 509) return "NW";
        if (adc < 553) return "NNE";
        if (adc < 616) return "N";
        if (adc < 681) return "SSW";
        if (adc < 747) return "SW";
        if (adc < 803) return "ENE";
        if (adc < 834) return "NE";
        if (adc < 880) return "SSE";
        if (adc < 914) return "S";
        if (adc < 941) return "ESE";
        if (adc < 972) return "SE";
        if (adc < 1010) return "E";
        else
        return ("ERR");   //  Error -- perhaps the sensor is disconnected?
}



boolean sendWeather()
{        
        unsigned int vbat = 0;      //  Battery voltage as read by FONA
        fona.getBattVoltage(&vbat); //  Read the battery voltage
        
        int solar;
        if (analogRead (A6) <= 900 && analogRead (A6) > 550)  //  Determine if panel is charging
        {
                solar = 1;
        }
        else
        {
                solar = 0;
        }
        
        /* We'll tell FONA to get ready for an HTTP GET command. We could use fona.HTTP_GET_START and build a url 
        with sprintf, but this uses a lot less SRAM because we can use flash strings, and sending floats is much
        easier.*/
        
        fona.sendCheckReply(F("AT+SAPBR=2,1"), F("OK"));
        fona.sendCheckReply(F("AT+SAPBR=1,1"), F("OK"));
        fona.sendCheckReply(F("AT+HTTPINIT"), F("OK"));
        fona.sendCheckReply(F("AT+HTTPPARA=\"CID\",1"), F("OK"));

        //  Now we'll construct the URL to transmit. sprintf is awkward with floats, so...
        fona.print(F("AT+HTTPPARA=\"URL\",\"http://data.sparkfun.com/input/"));
        fona.print(PUBLIC_KEY);
        fona.print(F("?private_key="));
        fona.print(PRIVATE_KEY);
        fona.print(F("&battery="));
        fona.print(vbat);
        fona.print(F("&humidity="));
        fona.print(humidity);
        fona.print(F("&lat="));
        fona.print(latitude,6);
        fona.print(F("&long="));
        fona.print(longitude,6);
        fona.print(F("&pressure="));
        fona.print(pressure);      
        fona.print(F("&rain_today="));
        fona.print(rain_today);
        fona.print(F("&temp="));
        fona.print(temp,1);
        fona.print(F("&wind_dir="));
        fona.print(wind_dir);
        fona.print(F("&wind_speed="));
        fona.print(wind_speed);    
        fona.println(F("\""));
        
        //  Echo to serial
        Serial.print(F("AT+HTTPPARA=\"URL\",\"http://data.sparkfun.com/input/"));
        Serial.print(PUBLIC_KEY);
        Serial.print(F("?private_key="));
        Serial.print(PRIVATE_KEY);
        Serial.print(F("&battery="));
        Serial.print(vbat);
        Serial.print(F("&humidity="));
        Serial.print(humidity);
        Serial.print(F("&lat="));
        Serial.print(latitude,6);
        Serial.print(F("&long="));
        Serial.print(longitude,6);
        Serial.print(F("&pressure="));
        Serial.print(pressure);      
        Serial.print(F("&rain_today="));
        Serial.print(rain_today);
        Serial.print(F("&temp="));
        Serial.print(temp,1);
        Serial.print(F("&wind_dir="));
        Serial.print(wind_dir);
        Serial.print(F("&wind_speed="));
        Serial.print(wind_speed);  
        Serial.println(F("\""));  
        
        flushFona();
        
        if(fona.sendCheckReply(F("AT+HTTPACTION=0"), F("OK")) == false)
        {  
                Serial.println(F("HTTPACTION failed."));
        }
        else
        {
                Serial.println(F("HTTPACTION succeeded."));
        }

        wait (5000);

        fona.println("AT+HTTPREAD");
        Serial.println(F("AT+HTTPREAD"));

        wait (5000);
        
        char fonaInBuffer[64];          //  For notifications from the FONA
        int httpMethod;
        int httpStatus;
        int dataLength;
        byte successFail = 99;
  
        fona.readBytesUntil ('\0', fonaInBuffer, 63);

        if (strstr (fonaInBuffer, "+HTTPACTION: 0,200,10") > 0)
        {
                Serial.println(F("Upload succeeded."));
                return true;
        }
        else
        {
                Serial.println(F("Upload appeared to fail. Buffer returned:"));
                Serial.println(fonaInBuffer);
                return false;
        }
}



void gpsFix()
{
        unsigned long timeLimit = millis() + 300000;    //  We'll attempt to get a fix for up to 5 minutes
        
        fona.enableGPS (true);
        
        Serial.println (F("Getting GPS fix..."));

        float lat;
        float lon;
        
        while(fona.GPSstatus() < 3 && millis() < timeLimit)
        {
                Serial.print (F("."));
        }

        if (fona.GPSstatus() >= 3)
        {
                Serial.println (F("Fix acquired. Homing in..."));
                wait (30000);   //  Wait for the fix to settle.
                fona.getGPS (&lat, &lon, NULL, NULL, &altitude);
        }
        else
        {
                Serial.println (F("GPS Fix timed out."));
        }

        latitude = lat;
        longitude = lon;

        Serial.print (latitude, 6);
        Serial.print (F(" "));
        Serial.print (longitude, 6);
        Serial.print (F(" "));
        Serial.println (altitude);
}



void getPressure()
{
        float rawPressure = bme.readPressure() / 100;
        float op1 = 1 - ((0.0065 * altitude) / (temp + 0.0065 * altitude + 273.15));
        float op2 = pow (op1, -5.257);
        pressure = rawPressure * op2;
}

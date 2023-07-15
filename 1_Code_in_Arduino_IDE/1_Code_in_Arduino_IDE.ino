#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>
#define buad 115200
SoftwareSerial ESP01(11, 10);
bool TempFlagWebPage = false;
char* LCD_INFO_PIR;
char* LCD_INFO_Smoke;
bool PIR_Flag_Detection = false;
bool Smoke_Flag_Detecton = false;
// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
void Timer0_Delay1ms() {
  TCCR0A |= (1 << WGM01);  // CTC
  TCCR0B |= 0x03;          //64 prescale
  OCR0A = 250;
  while ((TIFR0 & (1 << OCF0A)) == 0)
    ;
  TCCR0B = 0;
  TIFR0 = (1 << OCF0A);
}

bool Timer0_Delay_ms(int ms) {
  for (int i = 0; i < ms; i++) {
    Timer0_Delay1ms();
  }
  return true;
}


void Timer1_Init_Interrupt() {
  // Set up Timer 1 to generate an interrupt every 2 seconds
  cli();       // Disable interrupts . 34an kant btr34
  TCCR1A = 0;  // Set Timer 1 to normal mode
  TCCR1B = 0;  // Clear Timer 1 prescaler settings
  OCR1AH = 0x7A;
  OCR1AL = 0x12;                        // Set Timer 1 compare match value to 2 seconds
  TCCR1B |= (1 << WGM12);               // Set Timer 1 to CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set Timer 1 prescaler to 1024
  TIMSK1 |= (1 << OCIE1A);              // Enable Timer 1 interrupt
  sei();                                // Enable interrupts
}

ISR(TIMER1_COMPA_vect)  // Timer 1 Compare Match A ISR
{
  LCD_Output(PIR_READ_LCD(), Analog_Read(0), Smoke_READ_LCD());
  PIR_Flag_Detection = false;
}


char* PIR_READ_LCD() {
  if (PIR_Flag_Detection) {
    LCD_INFO_PIR = "motion detected";
  } else {
    LCD_INFO_PIR = "motion not detected";
  }
  return LCD_INFO_PIR;
}
char* Smoke_READ_LCD() {
  if (Smoke_Flag_Detecton) {
    LCD_INFO_Smoke = "Smoke";
  } else {
    LCD_INFO_Smoke = "NOSmoke";
  }
  return LCD_INFO_Smoke;
}

ISR(INT0_vect) {
  PIR_Flag_Detection = true;
}
ISR(INT1_vect) {
  if (Smoke_Flag_Detecton) {
    Smoke_Flag_Detecton = false;
    EICRA |= (1 << ISC11);  //Faling
    EICRA &= 0xFB;          //clear isc10

  } else {
    Smoke_Flag_Detecton = true;
    EICRA |= (1 << ISC11) | (1 << ISC10);  //rising
  }
}

int Analog_Read(uint8_t pin_num) {
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);  // //enable ADC and adjust clock Division Factor/128
  ADMUX |= pin_num;
  ADCSRA |= (1 << ADSC); // strart conversation
  while (ADCSRA & (1 << ADIF) == 0)
    ;
  float data = ADC;  // ADCH:ADCL
  data = (data * 5000 / 1024) / 10;
  //Serial.println(data);
  return (int)data;
}

void INT0_PIR_init(void) {
  // Configure PD2 (INT0 pin) as input with pull-up resistor enabled
  DDRD &= ~(1 << PD2); //input
  PORTD |= (1 << PD2); //pullup
  // Configure INT0 to trigger on rising edge
  EICRA |= (1 << ISC01) | (1 << ISC00);
  // Enable INT0 interrupt
  EIMSK |= (1 << INT0);
}

void INT1_Smoke_init(void) {
  // Configure PD3 (INT1 pin) as input with pull-up resistor enabled
  DDRD &= ~(1 << PD3);
  PORTD |= (1 << PD3);
  // Configure INT1 to trigger on falling edge
  EICRA |= (1 << ISC11);
  // Enable INT1 interrupt
  EIMSK |= (1 << INT1);
}

void LCD_Output(char* pir, int dergree, char* smoke) {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("TEMP");
  lcd.setCursor(5, 0);
  lcd.print(dergree);
  lcd.setCursor(8, 0);
  lcd.print("/");
  lcd.setCursor(9, 0);
  for (int i = 0; i < strlen(smoke); i++) {
    lcd.print(smoke[i]);
  }
  lcd.setCursor(0, 1);
  for (int i = 0; i < strlen(pir); i++) {
    lcd.print(pir[i]);
  }
}

String SendComandESP(String command, int times) {
  bool time = false;
  String response = "";
  ESP01.print(command);
  while (!time) {
    while (ESP01.available()) {
      char c = ESP01.read();
      response += c;
    }

    Serial.print(response);

    time = Timer0_Delay_ms(times);
  }


  return response;
}

void StartESP() {
  // SendComandESP("AT+RST\r\n",1000); //restart
  //SendComandESP("AT+RESTORE\r\n",10000); //reset setting
  SendComandESP("AT+UART_CUR=115200,8,1,0,0\r\n", 1000);
  SendComandESP("AT+CWJAP=\"Galaxy\",\"aaa15900\"\r\n", 5000);
  SendComandESP("AT+CWMODE=1\r\n", 2000);
  SendComandESP("AT+CIFSR\r\n", 2000);
  SendComandESP("AT+CIPMUX=1\r\n", 2000);
  SendComandESP("AT+CIPSERVER=1,80\r\n", 2000);
}

void Wifi_Send() {
  if (ESP01.available()) {
    if (ESP01.find("+IPD,")) {
      Timer0_Delay_ms(500);
      int connectionId = ESP01.read() - 48;
      // Read the request
      String request = "";
      while (ESP01.available()) {
        char c = ESP01.read();
        request += c;
      }
      Serial.println(request);
      // Check if the request is for LED control
      if (request.indexOf("led1_on") != -1) {
        PORTB |= (1 << PB4);
      } else if (request.indexOf("led1_off") != -1) {
        PORTB &= ~(1 << PB4);
      } else if (request.indexOf("led2_on") != -1) {
        PORTB |= (1 << PB5);
      } else if (request.indexOf("led2_off") != -1) {
        PORTB &= ~(1 << PB5);
      }
      if (Analog_Read(0) > 30) {
        TempFlagWebPage = true;
      } else {
        TempFlagWebPage = false;
      }
      // Send the response
      String response = "HTTP/1.1 200 OK\r\n";
      response += "Content-Type: text/html\r\n\r\n";
      response += "<html><head><title>LED Control</title></head><body><h1>Control the LEDs</h1><button onclick=\"sendCommand('led1_on')\">LED 1 On</button><button onclick=\"sendCommand('led1_off')\">LED 1 Off</button><br><br><button onclick=\"sendCommand('led2_on')\">LED 2 On</button><button onclick=\"sendCommand('led2_off')\">LED 2 Off</button><script>function sendCommand(command) {var xhttp = new XMLHttpRequest();xhttp.open('GET', '/'+command, true);xhttp.send();}</script></body></html>";
      if (TempFlagWebPage) {
        response += "<p>Temperature Warning!</p>";
      }
      // Check smoke flag and add warning message if necessary
      if (Smoke_Flag_Detecton) {
        response += "<p>Smoke Warning!</p>";
      }
      String cipSend = "AT+CIPSEND=";
      cipSend += connectionId;
      cipSend += ",";
      cipSend += response.length();
      cipSend += "\r\n";
      SendComandESP(cipSend, 500);
      ESP01.print(response);
      Timer0_Delay_ms(500);
      String closeCommand = "AT+CIPCLOSE=";
      closeCommand += connectionId;  // append connection id
      closeCommand += "\r\n";
      SendComandESP(closeCommand, 1000);
    }
  }
}
void setup() {
  DDRB |= (1 << PB4) | (1 << PB5); // leds output
  Timer1_Init_Interrupt();  //inturrpt timer 2s fixed
  Serial.begin(buad);
  ESP01.begin(buad);
  Timer0_Delay_ms(1000);
  StartESP();
  INT0_PIR_init();    // Initialize INT0 (PIR)
  INT1_Smoke_init();  // Initialize INT1 (Smoke)
}

void loop() {
  Wifi_Send();
}
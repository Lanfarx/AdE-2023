#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define myDigitalWrite(port, pin, value) if (value == HIGH ) ((port) |= (pin)); else ((port) &= ~(pin)) // Inserire ogni singolo bin del pin
#define myDigitalRead(port, pin) ((port & (1<<pin)) != 0) // Controlla il pin in base al numero non ai bit

bool luciPulsanti = 0, bothPressed = 0, greenButtonState = 0, redButtonState = 0, debounceGreen = 0, debounceRed = 0, allarme = 0, operativo = 0, ledAllarme = 0;
uint8_t greenButtonPressed = 4, redButtonPressed = 4, doubleClickTime = 400, debounce = 120;
uint16_t counterGreen = 0, counterRed = 0, timeCounter = 0, lightA = 0, lightB = 0;

void setup() {
  // put your setup code here, to run once:
  myDigitalWrite(DDRC, 0x0C, INPUT); // Impostato i Bottoni in input
  myDigitalWrite(DDRD, 0x0FC, INPUT); // Impostato gli Switch in input

  myDigitalWrite(PORTC, 0x0C, HIGH); // Impostato Pin Bottoni ad HIGH perchè quando premuti saranno LOW

  TCCR2A = 0; // Resetto i registri del Timer2
  TCCR2B = 0; // Resetto i registri del Timer2
  TCNT2 = 0; // Azzera Timer2
  myDigitalWrite(TCCR2B, 0x07, HIGH); // CTC mode Clear Timer on Compare Match
  myDigitalWrite(TCCR2B, 0x08, HIGH); // Prescaler 1024
  myDigitalWrite(TIMSK2, 0x02, HIGH); // Bit di confronto interrupt
  OCR2A = 16; // Controlla ogni 1 ms

  myDigitalWrite(PCICR, 0x06, HIGH); // Abilito la possibilità di interrupt per il PIN CHANGE
  myDigitalWrite(PCMSK1, 0x0C, HIGH); // Abilito gli interrupt nel gruppo 1 (PIN8-15)
  myDigitalWrite(PCMSK2, 0x1C, HIGH); // Abilito gli interrupt nel gruppo 2 (PIN16-23)

  TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1);  // Modalità Fast PWM
  TCCR1B = _BV(WGM12) | _BV(CS12);                 // 256 Prescaler

  sei();  // Abilita gli interrupt globali
}

void loop() {
  // put your main code here, to run repeatedly:
}

uint16_t readAnalogValue(byte pin) {
  ADMUX = (1 << REFS0) | (pin & 0x07);  // Imposta il riferimento e il pin
  ADCSRA |= (1 << ADEN) | (1 << ADSC);  // Abilita ADC e avvia la conversione
  while (ADCSRA & (1 << ADSC));         // Attendere il completamento
  uint16_t analogValue = ADC;           // Leggi il valore
  ADCSRA &= ~(1 << ADEN);               // Disabilita ADC
  return analogValue;
}

void gestionePulsanti(){ // 0 = waiting, 3 = click continuo, 4 = nessun click rilevato

  if(debounceRed && timeCounter == counterRed + debounce ){  // Controllo se ci sia da effettuare un debounce 
    if(redButtonState == 1 && myDigitalRead(PINC, 2) == 1) { debounceRed = 0; } // Controllo click continuo in caso non lo sia termino il debounce
      else counterRed = timeCounter; // Imposto il counter del pulsante rosso come il timecounter finché non viene rilasciato 
    if (!allarme && operativo && myDigitalRead(PINC, 2) == 0 && redButtonState == 0){ // Controllo se il pulsante viene premuto e se le luci sono accese
      redButtonPressed = 3;
      luciPulsanti = 1; // Abilito la modalità manuale di gestione delle luci
      lightA = lightA + 5; 
        if (lightA > 255) lightA = 255; 
    }
      else if (redButtonPressed == 3) redButtonPressed = 4; // Resetto il click
  }

  if(debounceGreen && timeCounter == counterGreen+debounce){  // Controllo se ci sia da effettuare un debounce
    if(greenButtonState == 1 && myDigitalRead(PINC, 3) == 1) { debounceGreen = 0; } // Controllo click continuo in caso non lo sia termino il debounce
      else counterGreen = timeCounter; // Imposto il counter del pulsante rosso come il timecounter finché non viene rilasciato
    if (!allarme && operativo && myDigitalRead(PINC, 3) == 0 && greenButtonState == 0){ // Controllo se il pulsante viene premuto e se le luci sono accese
      greenButtonPressed = 3;
      luciPulsanti = 1; // Abilito la modalità manuale di gestione delle luci
      lightA = lightA - 5;
        if (lightA > 255) lightA = 1;
    }
      else if (greenButtonPressed == 3) greenButtonPressed = 4; // Resetto il click
  } 

  if(bothPressed && timeCounter >= counterRed + debounce){ bothPressed = 0; }
  if(bothPressed && timeCounter >= counterGreen + debounce){ bothPressed = 0; }

  if(redButtonPressed == 0 && timeCounter == counterRed+debounce+doubleClickTime){ // Controllo se venga effettuato un click singolo
      redButtonPressed = 4;
  }
  if(greenButtonPressed == 0 && timeCounter == counterGreen+debounce+doubleClickTime){ // Controllo se venga effettuato un click singolo
       greenButtonPressed = 4;
  }
}

void gestioneLuci(){ // Simulo la gestione delle luci delle due stanze con due led
  if(operativo && !allarme && !luciPulsanti){ // Se in Operativo e non in Allarme gestisce le luci tramite i sensori di luminosità
    myDigitalWrite(DDRB, 0x06, OUTPUT);
    lightA = readAnalogValue(0x04);  // Leggi il valore analogico da A4
    lightB = readAnalogValue(0x05);  // Leggi il valore analogico da A5
    lightA = map(lightA, 0, 1023, 0, 255);  // Converte il range di valori di luminosità da un range di 0-1023 a 0-255 dato che sfrutta il PWM
    lightB = map(lightB, 0, 1023, 0, 255);  // Converte il range di valori di luminosità da un range di 0-1023 a 0-255 dato che sfrutta il PWM
    OCR1A = lightA; // Leggo i valori dal sensore di luminosità automaticamente per regolare le luci
    OCR1B = lightB; // Leggo i valori dal sensore di luminosità automaticamente per regolare le luci
  }
  else if (operativo && !allarme && luciPulsanti){ // Se tengo premuto uno dei pulsanti passo alla gestione manuale della luce
    OCR1A = lightA; // Leggo i valori che gestisco tramite i pulsanti 
    OCR1B = lightA; // Leggo i valori che gestisco tramite i pulsanti 
  }
  else { // Spengo la luce
    myDigitalWrite(DDRB, 0x06, LOW);
  }
}

void gestioneLed(){ 
  if(ledAllarme){  // Accendo il led di allarme se si apre una porta/finestra mentre in stato di allarme
    myDigitalWrite(PORTC, 0x02, HIGH);
  }
  else myDigitalWrite(PORTC, 0x02, LOW); 

  if(operativo){ // Accendo il led operativo se si è in stato operativo
    myDigitalWrite(PORTC, 0x01, HIGH);
  }
  else myDigitalWrite(PORTC, 0x01, LOW);
}

ISR (PCINT1_vect){ // Gestisco gli interrupt del gruppo 1 (PIN8-15) Pulsanti 
  if(greenButtonPressed == 0 && redButtonPressed == 0 ){ // Se interrupt da pulsante verde e contemporaneamente da rosso, imposta modalità allarme OFF e Operativo OFF (tutti i due led SPENTI)
    allarme = 0;
    ledAllarme = 0;
    operativo = 0;
    luciPulsanti = 0;
    bothPressed = 1;
  }
  
  if(!debounceRed && redButtonState && myDigitalRead(PINC, 2) == 0 && bothPressed == 0 && redButtonPressed == 4){ // Controlla la pressione singola del pulsante rosso
    counterRed = timeCounter;
    debounceRed = 1;
    redButtonPressed = 0;
  }
  if(!debounceRed && redButtonState && myDigitalRead(PINC, 2) == 0 && bothPressed == 0 && redButtonPressed == 0){ // Se interrupt da pulsante rosso e se doppio click breve, imposta modalità allarme ON
    counterRed = timeCounter;
    debounceRed = 1;
    redButtonPressed = 4;
    allarme = 1;
  }

  if(!debounceGreen && greenButtonState && myDigitalRead(PINC, 3) == 0 && bothPressed == 0 && greenButtonPressed == 4){ // Controlla la pressione singola del pulsante verde
    counterGreen = timeCounter;
    debounceGreen = 1;
    greenButtonPressed = 0;
  }
  if(!debounceGreen && greenButtonState && myDigitalRead(PINC, 3) == 0 && bothPressed == 0 && greenButtonPressed == 0){ // Se interrupt da pulsante verde e se doppio click breve, imposta modalità allarme OFF e Operativo ON
    counterGreen = timeCounter;
    debounceGreen = 1;
    greenButtonPressed = 4;
    allarme = 0;
    ledAllarme = 0;
    luciPulsanti = 0; // Resetta inoltre la gestione manuale delle luci tramite pulsanti
    operativo = 1;
  }
}

ISR (PCINT2_vect){ // Gestisco gli interrupt del gruppo 2 (PIN16-23) Switch
  if(allarme && myDigitalRead(PIND, 4) | myDigitalRead(PIND, 3) | myDigitalRead(PIND, 2) ){ 
    ledAllarme = 1;
  }
}

ISR(TIMER2_COMPA_vect) // Gestisco il Timer2
{
  TCNT2 = 0; // Azzera Timer
  timeCounter++;
  gestionePulsanti();
  gestioneLuci();
  gestioneLed();
  redButtonState = myDigitalRead(PINC,2); // Tengo traccia del precedente stato del pulsante rosso
  greenButtonState = myDigitalRead(PINC,3); // Tengo traccia del precedente stato del pulsante verde
}
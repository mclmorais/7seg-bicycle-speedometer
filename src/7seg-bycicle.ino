/*
 * Velocímetro para Bicicleta com Display 7 Segmentos
 * 
 * Este projeto implementa um sistema completo de monitoramento para bicicleta
 * que exibe informações em um display de 7 segmentos multiplexado.
 * 
 * Funcionalidades:
 * - Velocímetro: Exibe velocidade em km/h
 * - Odômetro: Distância total percorrida em metros
 * - Cronômetro: Tempo de viagem com pausa/retomar
 * - Monitor de Bateria: Nível de tensão da bateria
 * 
 * Hardware:
 * - Arduino (ATmega328P ou compatível)
 * - 4 dígitos de display 7 segmentos multiplexados
 * - Sensor Hall (pino 2) para detecção de ímãs na roda
 * - 2 botões para controle (pinos A0, A1)
 * - Shift register (74HC595) para controle do display
 * 
 * Pin Assignments:
 * - Pino 2: Sensor Hall (interrupção externa)
 * - Pino 4: Data (shift register)
 * - Pino 5: Clock (shift register)
 * - Pino 6: Multiplexação dígito 1
 * - Pino 7: Latch (shift register)
 * - Pino 9: Multiplexação dígito 2
 * - Pino 10: Multiplexação dígito 3 / Buzzer
 * - Pino 11: Multiplexação dígito 4
 * - Pino A0: Botão secundário
 * - Pino A1: Botão de operação
 */

// ============================================================================
// CONSTANTES - PADRÕES DE DISPLAY 7 SEGMENTOS
// ============================================================================

// Padrões binários para números 0-9 no display 7 segmentos
// Bit mapping: gfedcba (LSB primeiro)
const unsigned int sevenSegmentNumber[10] = {
  B11111100,  // 0
  B01100000,  // 1
  B11011010,  // 2
  B11110010,  // 3
  B01100110,  // 4
  B10110110,  // 5
  B10111110,  // 6
  B11100000,  // 7
  B11111110,  // 8
  B11100110   // 9
};

// Padrões para letras
const unsigned int sevenSegmentS = B10110111;  // Letra 'S' (Speedometer)
const unsigned int sevenSegmentC = B10011101;  // Letra 'C' (Chronometer)
const unsigned int sevenSegmentO = B11000111;  // Letra 'O' (Odometer)

// ============================================================================
// CONSTANTES - PINOS DE HARDWARE
// ============================================================================

const int dataPin = 4;              // Pino de dados do shift register
const int clockPin = 5;             // Pino de clock do shift register
const int latchPin = 7;             // Pino de latch do shift register
const int hallPin = 2;               // Sensor Hall (interrupção externa)
const int buzzerPin = 10;            // Buzzer (opcional)

const int buttonPins[2] = {A1, A0}; // Botões: [0]=Operação, [1]=Secundário
const int multiplexDigitPins[4] = {6, 9, 10, 11}; // Pinos de multiplexação dos dígitos

// ============================================================================
// CONSTANTES - PARÂMETROS FÍSICOS E CONFIGURAÇÃO
// ============================================================================

const int numberOfTeeth         = 8;                                  // Número de ímãs/dentes na roda
const double wheelRadius        = 0.34;                               // Raio da roda em metros
const double wheelCircumference = 2.0 * 3.14159 * wheelRadius;        // Circunferência em metros
const double distancePerTick    = wheelCircumference / numberOfTeeth; // Distância por tick (m)

// Constantes de conversão de velocidade
const double speedConversionFactor       = 7.690619;                                              // Fator de conversão para km/h
const double differentialSpeedMultiplier = 10000.0 * 2.0 * 3.14159 * wheelRadius / numberOfTeeth; // Multiplicador para cálculo diferencial

// Constantes de bateria (calibração para conversão de tensão)
const double batteryVoltageSlope  = 0.76923; // Coeficiente angular
const double batteryVoltageOffset = 2307.7;  // Offset linear

// Constantes temporais
const int samplePeriod                    = 1000; // Período de amostragem em ms
const int debounceDelay                   = 250;  // Delay de debounce em ms
const unsigned long longPressDuration     = 2000; // Duração de pressão longa em ms
const unsigned long letterDisplayDuration = 1000; // Duração de exibição de letra em ms
const unsigned long batteryReadInterval   = 1000; // Intervalo de leitura de bateria em ms

// Constantes de display
const unsigned long initialMultiplexDelay = 50000; // Delay inicial de multiplexação (microssegundos)
const unsigned long minMultiplexDelay     = 4400;  // Delay mínimo de multiplexação (microssegundos)
const int multiplexDelayDecrement         = 240;   // Decremento do delay na animação introdutória

// Constantes de timer
const unsigned int timerPrescaler         = 1024;  // Prescaler do Timer1
const unsigned int timerCompareValue      = 15624; // Valor de comparação para 1Hz (16MHz / 1024 / 1Hz - 1)

// Limites e máximos
const unsigned int maxMagnetTicks = 60000; // Máximo de ticks de ímã antes de overflow
const int maxDisplayValue         = 9999;  // Valor máximo exibível no display
const int numberOfReadings        = 1;     // Número de leituras para média móvel

// ============================================================================
// ENUMERAÇÕES
// ============================================================================

/**
 * Modos de operação do sistema
 */
enum OperationMode {
  SPEEDOMETER = 0,  // Velocímetro - exibe velocidade em km/h
  ODOMETER,         // Odômetro - exibe distância total em metros
  CHRONOMETER,      // Cronômetro - exibe tempo de viagem
  BATTERY           // Monitor de bateria - exibe tensão
};

/**
 * Estados do cronômetro
 */
enum ChronometerMode {
  RUNNING,  // Cronômetro em execução
  PAUSED    // Cronômetro pausado
};

// ============================================================================
// VARIÁVEIS GLOBAIS - ESTADO DO SISTEMA
// ============================================================================

OperationMode operationMode = BATTERY;           // Modo de operação atual
ChronometerMode chronometerMode = PAUSED;        // Estado do cronômetro

// ============================================================================
// VARIÁVEIS GLOBAIS - DISPLAY E MULTIPLEXAÇÃO
// ============================================================================

int currentDigit = 0x01;                         // Dígito atual sendo multiplexado (bit mask)
unsigned long multiplexDelay = initialMultiplexDelay; // Delay atual de multiplexação
unsigned long multiplexingTimer = 0;             // Timer para controle de multiplexação
boolean recentlyChangedOperation = false;        // Flag de mudança recente de modo
unsigned long letterDisplayTime = 0;             // Timestamp da última mudança de modo

// ============================================================================
// VARIÁVEIS GLOBAIS - ENTRADAS E BOTÕES
// ============================================================================

boolean buttonPressed[2] = {false, false};       // Estado dos botões [0]=Operação, [1]=Secundário
unsigned long debounceTime = 0;                  // Timestamp para debounce
unsigned long longPressTime = 0;                 // Timestamp do início de pressão longa

// ============================================================================
// VARIÁVEIS GLOBAIS - CÁLCULO DE VELOCIDADE
// ============================================================================

int magnetTicks = 0;                             // Contador de detecções de ímã
boolean magnetDot = false;                       // Estado do indicador visual de detecção
int circleIterator = 0;                          // Iterador para animação circular no velocímetro

// Método diferencial (velocidade instantânea)
unsigned long lastDifferentialTime = 0;           // Timestamp da última detecção
unsigned long currentDifferentialTime = 0;       // Timestamp da detecção atual
unsigned int differentialDelta = 0;             // Delta de tempo entre detecções (ms)
unsigned long differentialSpeed = 0;            // Velocidade calculada via método diferencial (km/h)

// Método de média (velocidade média)
int smoothingReadings[numberOfReadings];         // Array de leituras para média móvel
unsigned long readingTotal = 0;                  // Soma total das leituras
int readingIndex = 0;                            // Índice atual no array de leituras
unsigned long speedValue = 0;                    // Velocidade calculada via método de média (km/h)

// ============================================================================
// VARIÁVEIS GLOBAIS - ODOMETRO
// ============================================================================

double odometerValue = 0.0;                       // Distância total percorrida em metros

// ============================================================================
// VARIÁVEIS GLOBAIS - CRONÔMETRO
// ============================================================================

unsigned long chronometerStartTime = 0;          // Timestamp de início do cronômetro
unsigned long chronometerLastTime = 0;           // Timestamp da última atualização
unsigned long extraChronometerTime = 0;          // Tempo extra acumulado (para reset)
boolean chronometerJustReset = false;            // Flag indicando reset recente
unsigned long chronometerValue = 0;              // Valor do cronômetro formatado (MMSS)

// ============================================================================
// VARIÁVEIS GLOBAIS - BATERIA
// ============================================================================

unsigned long batteryTimer = 0;                  // Timer para controle de leitura de bateria
int lastVoltage = 0;                             // Última leitura de tensão em mV

// ============================================================================
// SETUP - INICIALIZAÇÃO DO HARDWARE
// ============================================================================

/**
 * Função de inicialização executada uma vez ao iniciar o Arduino
 */
void setup() {
  Serial.begin(9600);

  // Configuração dos pinos do shift register
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Configuração dos pinos de multiplexação
  for (int i = 0; i < 4; i++) {
    pinMode(multiplexDigitPins[i], OUTPUT);
  }

  // Configuração dos botões (pull-up interno)
  for (int i = 0; i < 2; i++) {
    digitalWrite(buttonPins[i], HIGH);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // Configuração da interrupção externa para sensor Hall
  attachInterrupt(digitalPinToInterrupt(hallPin), magnetDetected, CHANGE);

  // Configuração do Timer1 para interrupção periódica (1Hz)
  cli(); // Desabilita interrupções globais
  TCCR1A = 0;                    // Limpa registradores do Timer1
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = timerCompareValue;     // Valor de comparação para 1Hz
  TCCR1B |= (1 << WGM12);        // Modo CTC (Clear Timer on Compare)
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);       // Habilita interrupção de comparação
  sei(); // Habilita interrupções globais

  // Inicialização de timestamps
  lastDifferentialTime = millis();
  currentDifferentialTime = lastDifferentialTime;
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

/**
 * Loop principal do Arduino
 */
void loop() {
  intro();  // Executa animação introdutória
  core();   // Entra no loop principal de operação
}

// ============================================================================
// FUNÇÕES PRINCIPAIS DE OPERAÇÃO
// ============================================================================

/**
 * Animação introdutória do display
 * Cria efeito de fade-in reduzindo progressivamente o delay de multiplexação
 */
void intro() {
  while (multiplexDelay > minMultiplexDelay) {
    delayMicroseconds(multiplexDelay);
    multiplexDelay -= multiplexDelayDecrement;
    multiplexSevenSegment(15, 0x04); // Exibe "F" (ou padrão especial)
  }
  multiplexDelay = minMultiplexDelay; // Restaura delay mínimo
}

/**
 * Loop principal de operação do sistema
 * Polla botões e atualiza display conforme o modo atual
 */
void core() {
  while (true) {
    pollOperationButton();

    switch (operationMode) {
      case SPEEDOMETER:
        pollSpeedometerButton();
        // Exibe velocidade com ponto decimal no segundo dígito
        // e indicador visual no terceiro dígito quando ímã é detectado
        multiplexSevenSegment(differentialSpeed, 0x04 | (magnetDot ? 0x08 : 0x00));
        break;

      case ODOMETER:
        pollOdometerButton();
        // Exibe distância em metros (dividido por 10 para exibir em dezenas)
        // com ponto decimal no segundo dígito
        multiplexSevenSegment(((int)(odometerValue) / 10), 0x02);
        break;

      case CHRONOMETER:
        pollChronometerButton();
        // Exibe tempo no formato MM:SS
        multiplexSevenSegment(getChronometerValue(), getChronometerDecimals());
        break;

      case BATTERY:
        // Exibe nível de bateria calculado a partir da tensão
        readVcc(); // Atualiza lastVoltage se necessário
        multiplexSevenSegment(batteryVoltageSlope * lastVoltage - batteryVoltageOffset, 0x04);
        break;
    }
  }
}

// ============================================================================
// FUNÇÕES DE DISPLAY E MULTIPLEXAÇÃO
// ============================================================================

/**
 * Multiplexa o display de 7 segmentos com o número especificado
 * 
 * @param numberToDisplay Número a ser exibido (0-9999)
 * @param decimals Máscara de bits indicando quais dígitos devem mostrar ponto decimal
 *                 Bit 0 = dígito 1, Bit 1 = dígito 2, Bit 2 = dígito 3, Bit 3 = dígito 4
 */
void multiplexSevenSegment(short numberToDisplay, char decimals) {
  // Controle de taxa de atualização
  if ((micros() - multiplexingTimer) < multiplexDelay) {
    return;
  } else {
    multiplexingTimer = micros();
  }

  // Limita valor máximo exibível
  if (numberToDisplay > maxDisplayValue) {
    numberToDisplay = maxDisplayValue;
  }

  // Prepara shift register
  digitalWrite(latchPin, LOW);

  // Desabilita todos os dígitos
  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], LOW);
  }

  // Seleciona padrão para o dígito atual
  if ((currentDigit & 0x01) == 0x01) {
    // Primeiro dígito (mais significativo)
    if (recentlyChangedOperation && (millis() - letterDisplayTime < letterDisplayDuration)) {
      // Exibe letra indicando o modo por 1 segundo após mudança
      if (operationMode == SPEEDOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentS);
      } else if (operationMode == ODOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentO);
      } else if (operationMode == CHRONOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentC);
      } else if (operationMode == BATTERY) {
        shiftOut(dataPin, clockPin, LSBFIRST, 0xFF); // Todos os segmentos acesos
      }
    } else {
      recentlyChangedOperation = false;
      if (operationMode == SPEEDOMETER) {
        // No modo velocímetro, primeiro dígito mostra animação circular
        shiftOut(dataPin, clockPin, LSBFIRST, ~(1 << (circleIterator + 2)));
      } else {
        // Outros modos: exibe dígito mais significativo
        shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[numberToDisplay / 1000] | ((decimals & 0x01) == 0x01 ? 0x01 : 0x00)));
      }
    }
  } else if ((currentDigit & 0x02) == 0x02) {
    // Segundo dígito
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[(numberToDisplay / 100) % 10] | ((decimals & 0x02) == 0x02 ? 0x01 : 0x00)));
  } else if ((currentDigit & 0x04) == 0x04) {
    // Terceiro dígito
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[(numberToDisplay / 10) % 10] | ((decimals & 0x04) == 0x04 ? 0x01 : 0x00)));
  } else if ((currentDigit & 0x08) == 0x08) {
    // Quarto dígito (menos significativo)
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[numberToDisplay % 10] | ((decimals & 0x08) == 0x08 ? 0x01 : 0x00)));
  }

  // Atualiza display
  digitalWrite(latchPin, HIGH);

  // Habilita dígito atual
  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], (currentDigit & (1 << i)) == (1 << i));
  }

  // Avança para próximo dígito
  currentDigit = (currentDigit << 1);
  if (currentDigit > 0x08) {
    currentDigit = 0x01;
  }
}

// ============================================================================
// FUNÇÕES DE ENTRADA E BOTÕES
// ============================================================================

/**
 * Polla o botão de operação para troca de modo
 */
void pollOperationButton() {
  if (!buttonPressed[0] && !digitalRead(buttonPins[0])) {
    // Botão pressionado
    buttonPressed[0] = true;
    debounceTime = millis();
  } else if (buttonPressed[0] && digitalRead(buttonPins[0]) && (millis() - debounceTime) > debounceDelay) {
    // Botão solto após debounce
    changeOperationMode();
    buttonPressed[0] = false;
  }
}

/**
 * Polla o botão secundário no modo velocímetro
 * Alterna a animação circular no primeiro dígito
 */
void pollSpeedometerButton() {
  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    buttonPressed[1] = true;
    debounceTime = millis();
  } else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay) {
    circleIterator = (circleIterator + 1) % 6;
    buttonPressed[1] = false;
  }
}

/**
 * Polla o botão secundário no modo cronômetro
 * Pressione curto: pausa/retoma
 * Pressione longo (2s): reseta (quando pausado)
 */
void pollChronometerButton() {
  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    longPressTime = millis();
    buttonPressed[1] = true;
    debounceTime = millis();
  } else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay) {
    // Pressione curto: alterna pausa/retoma
    switch (chronometerMode) {
      case PAUSED:
        if (!chronometerJustReset) {
          chronometerStartTime += (millis() - chronometerLastTime);
          chronometerMode = RUNNING;
        } else {
          chronometerJustReset = false;
        }
        break;
      case RUNNING:
        chronometerMode = PAUSED;
        break;
    }
    buttonPressed[1] = false;
  } else if (buttonPressed[1] && !digitalRead(buttonPins[1])) {
    // Verifica pressão longa
    if (millis() - longPressTime >= longPressDuration) {
      if (chronometerMode == PAUSED) {
        // Reset apenas quando pausado
        extraChronometerTime = 0;
        chronometerStartTime = chronometerLastTime;
        chronometerJustReset = true;
      }
    }
  }
}

/**
 * Polla o botão secundário no modo odômetro
 * Pressione longo (2s): reseta a distância
 */
void pollOdometerButton() {
  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    longPressTime = millis();
    buttonPressed[1] = true;
    debounceTime = millis();
  } else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay) {
    buttonPressed[1] = false;
  } else if (buttonPressed[1] && !digitalRead(buttonPins[1])) {
    // Verifica pressão longa
    if (millis() - longPressTime >= longPressDuration) {
      odometerValue = 0.0;
    }
  }
}

/**
 * Alterna para o próximo modo de operação
 * Sequência: Velocímetro -> Odômetro -> Cronômetro -> Bateria -> Velocímetro
 */
void changeOperationMode() {
  recentlyChangedOperation = true;
  letterDisplayTime = millis();

  switch (operationMode) {
    case SPEEDOMETER:
      operationMode = ODOMETER;
      break;
    case ODOMETER:
      operationMode = CHRONOMETER;
      break;
    case CHRONOMETER:
      operationMode = BATTERY;
      break;
    case BATTERY:
      operationMode = SPEEDOMETER;
      break;
  }
}

// ============================================================================
// FUNÇÕES DE CÁLCULO DE VELOCIDADE
// ============================================================================

/**
 * Interrupção chamada quando sensor Hall detecta mudança (ímã)
 * Incrementa contador, atualiza odômetro e calcula velocidade diferencial
 */
void magnetDetected() {
  magnetTicks = (magnetTicks + 1) % maxMagnetTicks;
  circleIterator = (circleIterator + 1) % 6;
  magnetDot = !magnetDot;
  odometerValue += distancePerTick; // Incrementa distância percorrida
  calculateDeltaSpeed();
}

/**
 * Interrupção do Timer1 (1Hz)
 * Calcula velocidade média baseada em contagem de ticks durante período de amostragem
 */
ISR(TIMER1_COMPA_vect) {
  // Remove o valor mais antigo e adiciona o novo
  readingTotal -= smoothingReadings[readingIndex];
  smoothingReadings[readingIndex] = magnetTicks;
  readingTotal += smoothingReadings[readingIndex];

  readingIndex = ((readingIndex + 1) % numberOfReadings);

  // Cálculo de velocidade média:
  // - Multiplica por 10 para precisão decimal extra
  // - Divide pelo número de leituras para obter média
  // - Multiplica por fator de conversão para obter frequência de rotação
  // - Converte para km/h
  double doubleReading = ((double)readingTotal * 10) * speedConversionFactor / numberOfReadings * (1000 / samplePeriod) / numberOfTeeth;

  speedValue = ((int)doubleReading);

  magnetTicks = 0; // Reseta contador para próximo período
}

/**
 * Calcula velocidade instantânea usando método diferencial
 * Baseado no tempo entre detecções consecutivas de ímã
 */
void calculateDeltaSpeed() {
  currentDifferentialTime = millis();

  differentialDelta = currentDifferentialTime - lastDifferentialTime;
  lastDifferentialTime = currentDifferentialTime;

  // Fórmula: velocidade = (distância / tempo) * 3.6
  // onde distância = circunferência / número de dentes
  // e 3.6 converte m/s para km/h
  if (differentialDelta > 0) {
    differentialSpeed = (differentialSpeedMultiplier / differentialDelta) * 3.6;
  }

  Serial.println(differentialDelta);
}

// ============================================================================
// FUNÇÕES DE CRONÔMETRO
// ============================================================================

/**
 * Calcula e retorna o valor do cronômetro formatado como MMSS
 * 
 * @return Valor do cronômetro em formato MMSS (ex: 1234 = 12:34)
 */
int getChronometerValue() {
  unsigned long chronometerRawValue = extraChronometerTime;

  switch (chronometerMode) {
    case PAUSED:
      // Quando pausado, usa tempo acumulado até última pausa
      chronometerRawValue += chronometerLastTime - chronometerStartTime;
      break;
    case RUNNING:
      // Quando rodando, calcula tempo atual
      chronometerRawValue += millis() - chronometerStartTime;
      chronometerLastTime = millis();
      break;
  }

  // Converte para minutos e segundos
  unsigned long minutes = chronometerRawValue / 60000;
  chronometerValue = minutes * 100;

  unsigned long seconds = (chronometerRawValue / 1000) % 60;
  chronometerValue += seconds;

  return chronometerValue;
}

/**
 * Retorna máscara de decimais para exibição do cronômetro
 * Inclui dois pontos (:) entre minutos e segundos
 * 
 * @return Máscara de bits para pontos decimais
 */
int getChronometerDecimals() {
  // Bit 1 (0x02) = dois pontos após minutos
  // Bit 3 (0x08) = ponto após segundos (pisca quando rodando)
  return 0x02 | (chronometerMode == RUNNING ? 0x08 : 0x00);
}

// ============================================================================
// FUNÇÕES DE BATERIA
// ============================================================================

/**
 * Lê a tensão de alimentação (Vcc) usando referência interna de 1.1V
 * 
 * @return Tensão em milivolts (mV)
 */
long readVcc() {
  if (millis() - batteryTimer > batteryReadInterval) {
    batteryTimer = millis();

    // Configura ADC para ler referência interna de 1.1V contra Vcc
    // Configuração varia conforme microcontrolador AVR
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    // ATmega328P e similares
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Aguarda estabilização da referência
    ADCSRA |= _BV(ADSC); // Inicia conversão
    while (bit_is_set(ADCSRA, ADSC)); // Aguarda conclusão

    // Lê resultado (deve ler ADCL primeiro)
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    long result = (high << 8) | low;

    // Calcula Vcc em mV
    // Fórmula: Vcc = (1.1V * 1023 * 1000) / leitura_ADC
    // Ajuste calibrado: 1105508 = 1.1 * (3.91/3.98) * 1023 * 1000
    result = 1105508 / result;
    lastVoltage = result; // Atualiza última leitura

    return result; // Retorna Vcc em milivolts
  }
  return lastVoltage; // Retorna última leitura se ainda não passou o intervalo
}

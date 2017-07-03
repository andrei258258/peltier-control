// GELA PELE 3000
// Trabalho final de instrumentação 2017
// Alunos: Andrei Donati e Fernando Battisti

//DEFINIÇÕES DO PROGRAMA

#define PELTIER 9
#define COOLER 10
#define NTC1 3
#define POT 0
#define NUMAMOSTRAS 8  // numero de amostras do sensor para fazer a média
#define BUTTON1 2
#define LED1 13 // led's usados para sinalizar o estado do equipamento 
#define LED2 8
#define TEMPO_COOLER 5000 // tempo do cooler ligado após o desligamento do peltier

// Include de libraries 
#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Endereço do lcd --> 63 (0x3F)  

// Variáveis
int val_pot;
byte estado = 3;   // Usada para simular uma máquina de estados. Se estado = 3 -> Configuração   ||   Se estado = 1 -> loop de controle || Se estado = 2 -> Configuração com coolers ligados
byte temp_SP = 10;   // Set point de temperatura que poderá ser alterado pelo usuário
float temperatura= 20; // Temperatura atual
float erro = 10; // Erro atual
float erro_passado=10; // Erro anterior, usado para fazer o controle
float acao_controle =255; // Valor da ação de controle, em PWM já
float saida_passada= 255 ; // Saida de controle anterior, usada para fazer o controle
float saida_passada_sem_sat = 255; // valor da ação de controle, em PWM já
boolean controle_coolers =0; 
int amostra[NUMAMOSTRAS]; // vetor de amostras 
unsigned long tempo_agora;
unsigned long tempo_passado; 

//Parâmetros de calibração do NTC
float pad = 9850;        // Resistor utilizado no divisor de tensão
float thermr = 10000;    // Resistência do NTC a 25º
float a = 0.001109148;   // Estes parâmetros a, b, c e d são usados para ajustar o NTC conforme calibração
float b = 0.000224315;
float c = 4.0829210E-06;
float d = 0.0000000876741; 

byte custom[8] = { // Caractere customizado. Simbolo de grau
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};



// INICIALIZAÇÃO DO SISTEMA
void setup(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(COOLER, OUTPUT);
  pinMode(PELTIER, OUTPUT);
  
  Serial.begin(115200);
  
  lcd.begin(16,2);        // LCD de 16 caracteres por 2 linhas
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("-- WELCOME TO --");
  lcd.setCursor(0,1);
  lcd.print(" GELA PELE 3000");
  lcd.createChar(5, custom);    // Cria o caractere customizado 

  attachInterrupt(0, muda_estado,RISING); // Define a interrupção que muda o estado do sistema

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(1000);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  delay(1000);

  //Modificar a frequencia dos PWMs dos pinos 9 e 10 para 30,5 
  byte mode;
  mode = 0x05; // 30,5 Hz
  TCCR1B = TCCR1B & 0b11111000 | mode;
  delay(3000);
}



// LOOP DO SISTEMA
void loop(){
  
    tempo_agora=millis();
    temp_SP = f_temp_sp(); // Lê temperatura de set-point setada através do potênciometro 

    if (estado == 1){ // estado de atuação com loop de controle 
        
      lcd_estado_1(); //Imprime no LCD os valores de referencia e temp atual

      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(COOLER, HIGH);
      
      // Loop de controle
          temperatura = le_temp(NTC1); // leitura de temperatura atual
          erro = temp_SP - temperatura; // calculo do erro
          acao_controle= calcula_controle(erro, erro_passado, saida_passada); // calcula a ação de controle, já em valores do PWM
          analogWrite(PELTIER, acao_controle); // faz a atuação do controle calculado
          erro_passado = erro; // atualiza as variáveis
          saida_passada= acao_controle;

    //Acaba loop de controle 

   }
   else if(estado == 2){ // estado para deixar coolers ligados apos acabar o tempo
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(COOLER,HIGH);
      analogWrite(PELTIER, 0); 
      lcd_estado_0();       // Imprime no LCD as instruções
      if((tempo_agora-tempo_passado) > TEMPO_COOLER){// passa para o estado de configuração depois do tempo definido
        estado= 3;
      }
  }
  else{ // estado de configuração
      digitalWrite(LED1, HIGH); // leds de informação do usuário
      digitalWrite(LED2, LOW);
      lcd_estado_0();       // Imprime no LCD as instruções
      digitalWrite(COOLER,LOW); // deslida os coolers e o peltier
      analogWrite(PELTIER, 0); 
  }
  
  delay(63); // taxa de amostragem de 100 ms
 
}




// FUNÇÕES
float le_temp(int ThermistorPIN) { // esta função faz a leitura analógica algumas vezes e depois faz a média destas medidas. O sistema ficou mais robusto desta forma
  int i;
  int ADC;
  long Resistance;  
  float Temp;
  float media;  
   
  for (i=0; i< NUMAMOSTRAS; i++) {
     amostra[i] = analogRead(ThermistorPIN);
     delay(5); // tempo para descarga de capacitores
  }
  media = 0;
  for (i=0; i< NUMAMOSTRAS; i++) {
     media += amostra[i];
  }
  ADC= media / NUMAMOSTRAS; // faz a média das medidas 
  Resistance =pad*((1024.0 / ADC) - 1);  // transforma do valor de leitura para a resitencia 

  // Calculo da temperatura através da equação que descreve o NTC
  Temp = log(Resistance); // Salvando o log de R evitamos de calcular ela 3 vezes
  Temp = 1 / (a + (b * Temp) + (c * Temp * Temp) + (d * Temp * Temp * Temp)); //Equação de Steinhart–Hart
  Temp = Temp - 273.15;  // Kelvin para Celsius                      
  return Temp;
}

int calcula_controle(float erro,float erro_passado,float saida_passada){
  float a= -0.06674;
  float b = 0.05339;
  float valor = 0;
  
  saida_passada = map(saida_passada, 0, 255, 0, 11.5); // mapping para voltar aos parâmetros de valores de 0 a 11.5
  valor = saida_passada + a*erro - b*erro_passado; // equação de um controlador PI. A equação já usa o valor da saida passada saturada, fazendo 
                                                   // espécie de anti-windup implicito 
                                                    
  if(valor>11.5){ valor=11.5; } // saturação do controle
  if(valor<0){ valor=0; }
  
  valor = map(valor, 0, 11.5, 0, 255); // mapping para voltar aos parâmetros de valores de 0 a 255, necessários ao PWM
  return valor;
}


byte f_temp_sp (){ // retorna o valor da temperatura desejada de acordo com o valor lido do potenciometro
  val_pot = 1023 - analogRead(POT);
  byte val_pot_mapeado = map(val_pot, 0, 1023, 2, 12);
  return val_pot_mapeado;
}

void lcd_estado_0(){ // imprime informações no lcd no estado 2 e 3
  lcd.setCursor(0,0);
  lcd.print("Aperte p/ ligar     ");
  lcd.setCursor(0,1);
  lcd.print("Temp desej: ");
  lcd.print(temp_SP);
  lcd.write(5);      // caracter custom
  lcd.print("C              ");

}

void lcd_estado_1(){ // imprime informações no lcd no estado 1
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SP: ");
  lcd.print(temp_SP);
  lcd.write(5);                 // Caracter custom
  lcd.print("C              ");
  lcd.setCursor(0,1);
  lcd.print("VP: ");
  int temperatura_inteira=0;
  temperatura_inteira= temperatura;
  lcd.print(temperatura_inteira);
  lcd.write(5);                 // Caracter custom
  lcd.print("C"); 
  lcd.print("--U: ");
  int tensao = 0;
  tensao = map(saida_passada, 0, 255, 0, 11.5);
  lcd.print(tensao);
  lcd.print("V     "); 
}

void muda_estado(){ //função que altera o estado do sistema
  delay(900);
  if(estado == 3) { estado = 1; } // regras de mudanças de estados
  else if(estado == 1) { estado = 2; }
  else if(estado == 2) { estado = 1; }
  tempo_passado = millis();
  delay(900);
}

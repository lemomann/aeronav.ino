#include <Wire.h>
#include <SPI.h>
#include <SD.h>
const int MPU = 0x68;
const int chipSelect = 4;
float AccX, AccX_rol[5], AccX_sum, AccX_cal, AccY, AccY_rol[5], AccY_sum, AccY_cal, AccZ, AccZ_rol[5], AccZ_sum, AccZ_cal, Acc_cal_mod, cal_iter, Acc_mod;
float GyroX, GyroX_sum, GyroX_rol[5], GyroX_cal, GyroY, GyroY_sum, GyroY_rol[5], GyroY_cal, GyroZ, GyroZ_sum, GyroZ_rol[5], GyroZ_cal;
float grav[3], omegaX, omegaY, omegaZ, grav_mod_pxy, grav_mod_pxz, grav_mod_pyz, grav_ang_xy, grav_ang_yz, grav_ang_zx, cal_fac;
int i, flip_flag_xy, flip_flag_yz, flip_flag_zx;
unsigned long t;

void setup() {

  // put your setup code here, to run once:
  cal_iter = 200;
  flip_flag_xy = 0;
  flip_flag_yz = 0;
  flip_flag_zx = 0;
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                   
  Wire.endTransmission(true);
  for (i=0;i<=cal_iter;i++) // loop de calibração, acelerometro
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX_sum += (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81; // X-axis value. multiplicando por 9.81 para obter aceleração no SI
    AccY_sum += (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81;// Y-axis value
    AccZ_sum += (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81; // Z-axis value
    Serial.print(i/cal_iter*100.0);
    Serial.println("%");
  }

  for (i=0;i<=cal_iter;i++) // loop de calibração, giroscopio
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
    GyroX_sum += (Wire.read() << 8 | Wire.read()); // X-axis value
    GyroY_sum += (Wire.read() << 8 | Wire.read());// Y-axis value
    GyroZ_sum += (Wire.read() << 8 | Wire.read()); // Z-axis value
    Serial.print(i/cal_iter*100.0);
    Serial.println("%");
  }
// Acc_sum são floats, mas Gyro_sum são ints. isso porque trabalhamos com a média das acelerações, dividindo acc_sum por cal_iter,
//enquanto gyro_sum é somente um offset a ser subtraído
  

  AccX_cal = AccX_sum/cal_iter;
  AccY_cal = AccY_sum/cal_iter;
  AccZ_cal = AccZ_sum/cal_iter;
  Acc_cal_mod = sqrt(pow(AccX_cal,2)+pow(AccY_cal,2)+pow(AccZ_cal,2));
  cal_fac = 9.81 / Acc_cal_mod; //fator de calibração
  Serial.println(Acc_cal_mod);
  grav[0] = AccX_cal * cal_fac;
  grav[1] = AccY_cal * cal_fac;
  grav[2] = AccZ_cal * cal_fac;



  //Acc_cal_mod = 1.05; DEBUG
  for (i=0;i<=4;i++) // preenche as listas pela primeira vez
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX_rol[i] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac; // X-axis value
    AccY_rol[i] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac;// Y-axis value
    AccZ_rol[i] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac; // Z-axis value
  }

  GyroX_cal = GyroX_sum/cal_iter;
  GyroY_cal = GyroY_sum/cal_iter;
  GyroZ_cal = GyroZ_sum/cal_iter;

  for (i=0;i<=4;i++) // preenche as listas pela primeira vez
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    GyroX_rol[i] = ((Wire.read() << 8 | Wire.read()) - GyroX_cal) / 131.0;
    GyroY_rol[i] = ((Wire.read() << 8 | Wire.read()) - GyroY_cal) / 131.0;
    GyroZ_rol[i] = ((Wire.read() << 8 | Wire.read()) - GyroZ_cal) / 131.0;
  }

  Serial.print(GyroX_cal);
  Serial.print(",");
  Serial.print(GyroY_cal);
  Serial.print(",");
  Serial.println(GyroZ_cal);
  Serial.print("cal_fac = ");
  Serial.println(cal_fac);

  Serial.println("pronto");
  /*while (Serial.available() == 0)
  {}*/


  t=millis();
  grav_ang_xy = acos(grav[0]/sqrt(pow(grav[0],2)+pow(grav[1],2)));
  grav_ang_yz = acos(grav[1]/sqrt(pow(grav[1],2)+pow(grav[2],2)));
  grav_ang_zx = acos(grav[2]/sqrt(pow(grav[2],2)+pow(grav[0],2)));
}

void loop() {
  // put your main code here, to run repeatedly:

  String dataString = "";
  int AccX, AccY, AccZ, endFlag;

  if (Serial.available()!=0)
  {
    endFlag = 1;
    Serial.print("ended!");
    while (endFlag == 1)
    {}
  }
  
  for (i=0;i<=3;i++) //desce todos os valores uma posição na lista. O mais antigo é apagado
  {
    AccX_rol[i] = AccX_rol[i+1];
    AccY_rol[i] = AccY_rol[i+1];
    AccZ_rol[i] = AccZ_rol[i+1];
    GyroX_rol[i] = GyroX_rol[i+1];
    GyroY_rol[i] = GyroY_rol[i+1];
    GyroZ_rol[i] = GyroZ_rol[i+1];
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX_rol[4] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac; // X-axis value
  AccY_rol[4] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac;// Y-axis value
  AccZ_rol[4] = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 * cal_fac; // Z-axis value. Escreve a leitura da aceleração 
  //mais recente na última posição das listas

  AccX_sum = 0.0;
  AccY_sum = 0.0;
  AccZ_sum = 0.0; //reinicia todas as somas
  GyroX_sum = 0.0;
  GyroY_sum = 0.0;
  GyroZ_sum = 0.0;


  
  for (i=0;i<=4;i++) //soma para calcular a média
  {
    AccX_sum += AccX_rol[i];
    AccY_sum += AccY_rol[i];
    AccZ_sum += AccZ_rol[i];
    GyroX_sum += GyroX_rol[i];
    GyroY_sum += GyroY_rol[i];
    GyroZ_sum += GyroZ_rol[i];
  }

  AccX = AccX_sum/5.0; //média das acelerações
  AccY = AccY_sum/5.0;
  AccZ = AccZ_sum/5.0;
  Acc_mod = sqrt(pow(AccX,2)+pow(AccY,2)+pow(AccZ,2)); //módulo das acelerações médias
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX_rol[4] = ((Wire.read() << 8 | Wire.read()) - GyroX_cal) / 131.0;
  GyroY_rol[4] = ((Wire.read() << 8 | Wire.read()) - GyroY_cal) / 131.0;
  GyroZ_rol[4] = ((Wire.read() << 8 | Wire.read()) - GyroZ_cal) / 131.0; //Escreve a leitura da velocidade de giro
  //mais recente na última posição das listas

  omegaX = GyroX_rol[4]*3.1415/180.0*(millis()-t)/1000.0;
  omegaY = GyroY_rol[4]*3.1415/180.0*(millis()-t)/1000.0;
  omegaZ = GyroZ_rol[4]*3.1415/180.0*(millis()-t)/1000.0; //calcula o ângulo de giro no intervalo de tempo. uma integração discreta
  t = millis(); //começa a contar o tempo para o próximo intervalo

  grav_mod_pxy = sqrt(pow(grav[0],2)+pow(grav[1],2));
  grav_mod_pxz = sqrt(pow(grav[0],2)+pow(grav[2],2));
  grav_mod_pyz = sqrt(pow(grav[1],2)+pow(grav[2],2)); //módulos das projeções do vetor gravidade nos planos coordenados

  grav_ang_xy = acos(grav[0]/sqrt(pow(grav[0],2)+pow(grav[1],2))) + omegaZ;
  grav_ang_yz = acos(grav[1]/sqrt(pow(grav[1],2)+pow(grav[2],2))) + omegaX;
  grav_ang_zx = acos(grav[2]/sqrt(pow(grav[2],2)+pow(grav[0],2))) + omegaY; //ângulos após a rotação

  grav[0] = sqrt(pow(Acc_cal_mod,2)/(1/pow(cos(grav_ang_yz),2)*(-1+1/pow(cos(grav_ang_xy),2))+1));
  grav[1] = sqrt(pow(Acc_cal_mod,2)/(1/pow(cos(grav_ang_zx),2)*(-1+1/pow(cos(grav_ang_yz),2))+1));
  grav[2] = sqrt(pow(Acc_cal_mod,2)/(1/pow(cos(grav_ang_xy),2)*(-1+1/pow(cos(grav_ang_zx),2))+1));

  

  /*grav_next[0] = -grav[0] + grav_mod_pxy * cos(grav_ang_xy - omegaZ) + grav_mod_pxz * cos(grav_ang_zx - omegaY);
  grav_next[1] = -grav[1] + grav_mod_pyz * cos(grav_ang_yz - omegaX) + grav_mod_pxy * cos(grav_ang_xy - omegaZ);
  grav_next[2] = -grav[2] + grav_mod_pxz * cos(grav_ang_zx - omegaY) + grav_mod_pyz * cos(grav_ang_yz - omegaX);

  //grav_ang_xy = angulo que a projeção do vetor gravidade em xy faz com o eixo x
  //grav_ang_yz = angulo que a projeção do vetor gravidade em yz faz com o eixo y
  //grav_ang_zx = angulo que a projeção do vetor gravidade em zx faz com o eixo z

  A fórmula acima é uma tentativa de computar as novas componentes do vetor gravidade após as rotações omega. não está funcionando.
  Vou agora tentar empregar a teoria de rotações de Euler, que postula que toda rotação de um corpo extenso pode ser decomposta em no máximo
  três rotações sucessivas, em torno de três eixos coordenados. Dado que nosso referencial é o sensor, e não um referencial externo, as rotações devem ser
  tratadas como extrínsecas, onde o vetor gravidade gira em torno de um referencial estático, e não o contrário. */

  //computando a rotação devido a omegaX

  //grav[0] mantém-se inalterado

  /*grav_next[1] = sqrt(pow(grav[1],2)+pow(grav[2],2)) * cos(omegaX + acos(grav[1]/sqrt(pow(grav[1],2)+pow(grav[2],2))));
  grav_next[2] = sqrt(pow(Acc_cal_mod,2)-pow(grav[0],2)-pow(grav_next[1],2));

  grav_ang_xy = acos(grav[0]/sqrt(pow(grav[0],2)+pow(grav_next[1],2)));
  grav_ang_yz = acos(grav[1]/sqrt(pow(grav_next[1],2)+pow(grav_next[2],2)));
  grav_ang_zx = acos(grav[2]/sqrt(pow(grav_next[2],2)+pow(grav[0],2)));*/


  //muda o sinal das componentes caso estejam se aproximando de zero
  //quando grav_ang_xy muda de sinal, grav[1] muda de sinal
  //if (grav_ang_xy <= 0.01 && omegaZ <= 0.0)
  if (grav_ang_xy <= 0.0)
  {
    flip_flag_xy = 1;
  }
  if (flip_flag_xy == 1)
  {
    grav[1] = -grav[1];
    //if (grav_ang_xy >= -0.01 && omegaZ >= 0.0)
    if (grav_ang_xy >= 0.0)
    {
      flip_flag_xy = 0;
    }
  } 

  //TODO: aplicar o protocolo de mudança de sinal aos demais componentes; repetir os passos acima para os demais omegas; testar

  //quando grav_ang_yz muda de sinal, grav[2] muda de sinal
  //if (grav_ang_yz <= 0.01 && omegaX <= 0.0)
  if (grav_ang_yz <= 0.0)
  {
    flip_flag_yz = 1;
  }
  if (flip_flag_yz == 1)
  {
    grav[2] = -grav[2];
    //
    //if (grav_ang_yz >= -0.01 && omegaX >= 0.0)
    if (grav_ang_yz >= 0.0)
    {
      flip_flag_yz = 0;
    }
  }

  //quando grav_ang_zx muda de sinal, grav[0] muda de sinal
  //if (grav_ang_zx <= 0.01 && omegaY <= 0.0)
  if (grav_ang_zx <= 0.0)
  {
    flip_flag_zx = 1;
  }
  if (flip_flag_zx == 1)
  {
    grav[0] = -grav[0];
    //if (grav_ang_zx >= -0.01 && omegaY >= 0.0)
    if (grav_ang_zx >= 0.0)
    {
      flip_flag_zx = 0;
    }
  }


  Serial.print(grav[0]);
  Serial.print(",");
  Serial.print(grav[1]);
  Serial.print(",");
  Serial.print(grav[2]);
  Serial.print("||");
  Serial.print(grav_ang_xy*180.0/3.1415);
  Serial.print(",");
  Serial.print(grav_ang_yz*180.0/3.1415);
  Serial.print(",");
  Serial.print(grav_ang_zx*180.0/3.1415);
  Serial.print("||");
  Serial.print(flip_flag_xy);
  Serial.print(",");
  Serial.print(flip_flag_yz);
  Serial.print(",");
  Serial.println(flip_flag_zx);


  if (isnan(grav[0]))
  {
    Serial.print("overflow!");
    while(1)
    {}
  }


}

/*
 * File:          my_controller.c
 * Date: 18/11/2020
 * Description: Controlador do robo AstroBoy
 * Author: Guilherme H. Moreira
 * Modifications:
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define QtddSensoresProx 8
#define QtddLeds 10
#define dife 0.01


int main(int argc, char **argv) {
  

  int i=0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  const double *posicao; //variáel que vai receber a posição do robo
  
  double x[250]; // vetor que armazenas a posição do X quando ocorre colisão
  double z[250]; // vetor que armazenas a posição do Z quando ocorre colisão
  int contled = 0;  //Flag para acionar os LED em caso de colisao com movimentacao
  int ledac = 0;   //acende led especifico a cada colisao
  double valpos; //variavel para positivos
  double valneg; //variavel para negativos

  int vetContr = 0;  // flag que controla o armazenamento nos vetores x e z
  int moreCol = 0;  // flag para aumentar o tempo de colisão (8 iteracoes do while)
  
  
  
  for(i=0;i<257;i++) texto[i]='0';
  
  wb_robot_init();
  
  //configurei MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);
  
  
   //configurei Sensores de Proximidade
   WbDeviceTag SensorProx[QtddSensoresProx];
   
   SensorProx[0] = wb_robot_get_device("ps0");
   SensorProx[1] = wb_robot_get_device("ps1");
   SensorProx[2] = wb_robot_get_device("ps2");
   SensorProx[3] = wb_robot_get_device("ps3");
   SensorProx[4] = wb_robot_get_device("ps4");
   SensorProx[5] = wb_robot_get_device("ps5");
   SensorProx[6] = wb_robot_get_device("ps6");
   SensorProx[7] = wb_robot_get_device("ps7");
   
   wb_distance_sensor_enable(SensorProx[0],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[1],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[2],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[3],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[4],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[5],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[6],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[7],TIME_STEP);
  
   WbNodeRef robot_node = wb_supervisor_node_get_from_def("Astroboy"); //captura o supervisor
   WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição
   
    //config leds
    WbDeviceTag Leds[QtddLeds];
    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    Leds[8] = wb_robot_get_device("led8");
    Leds[9] = wb_robot_get_device("led9");

   
   
   //loop de iteração do Astroboy

  while (wb_robot_step(TIME_STEP) != -1) {
    for(i=0;i<256;i++) texto[i]=0;
    //memcpy(texto,0,10);
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       sprintf(texto,"%s|%d: %5.2f  ",texto,i,LeituraSensorProx[i]);
       posicao = wb_supervisor_field_get_sf_vec3f(trans_field);

    }
    printf("%s\n",texto);
    printf("Posicao do Astroboy: x= %f   y= %f z= %f\n", posicao[0], posicao[1], posicao[2]);
    
    //if sensor nao esta proximo anda para frente
    if(LeituraSensorProx[0]>350){
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
      
      x[vetContr] = posicao[0]; // atualiza o vetor de posicao x[]
      z[vetContr] = posicao[2]; // atualiza o vetor de posicao z[] 
      printf("Astroboy colidiu em: x%d %f | z%d %f\n", vetContr,  x[vetContr], vetContr, z[vetContr]); //printa colisao
      
      if(moreCol > 9){  //Se a flag de aumento de colisao for menor que 9, entao continua colidindo
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
        //Tratamento: 
        //se a posicao do robo em X for MAIOR que 0 
        if(x[0] > 0){

          //se x[2] for maior que x[8]
          if(x[3] > x[9]){
            valpos = x[3] - x[9];           //distância     
            if(valpos > 0.015){             //se distância maior que 0.01 entao o objeto foi movimentado
               printf("[-][-][-][-][-][-]  MOVIMENTACAO DETECTADA PELO SENSOR S0 [-][-][-][-][-][-]\n");
               contled = contled + 2500;    //flag para acender o LED
            }
          }
          //se x[8] for maior que x[2]
          else{
            valpos = x[9] - x[3];           //distância
            if(valpos > 0.015){             //se distância maior que 0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-] MOVIMENTACAO DETECTADA PELO SENSOR S0 [-][-][-][-][-][-]\n");
              contled = contled + 2500;     //flag para acender o LED
            }
          }  
        }
        //Tratamento: 
        //se a posicao do robo em X for MENOR que 0 
        else if(x[0] < 0){ 
            //se x[2] for maior que x[8]
            if(x[3] > x[9]){ 
                valneg = (x[9]) - (x[3]);   //distância    
            if(valneg < -0.015){            //se distância menor que -0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-] MOVIMENTACAO DETECTADA PELO SENSOR S0 [-][-][-][-][-][-]\n");
               contled = contled + 2500;    //flag para acender o LED
            }
          }

          else{
            valneg = (x[3]) - (x[9]);       //distância
            if(valneg < -0.015){            //se distância menor que -0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-] MOVIMENTACAO DETECTADA PELO SENSOR S0 [-][-][-][-][-][-]\n");
               contled = contled + 2500;    //flag para acender o LED
            }
          }  
        }
      }
      
      vetContr++;     //Itera o controlador de armazenamento nos vetores x e z
      moreCol++;     //Itera a flag de aumento de colisao
     }

     //if sensor 7 está proximo
     else if(LeituraSensorProx[7]>350) {
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
     
      x[vetContr] = posicao[0]; // atualiza o vetor de posicao x[]
      z[vetContr] = posicao[2]; // atualiza o vetor de posicao z[]
      printf("Astroboy colidiu em: x%d %f | z%d %f\n", vetContr,  x[vetContr], vetContr, z[vetContr]); //printa colisao
      
     if(moreCol > 9){  //Se a flag de aumento de colisao for menor que 9, entao continua colidindo
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
         //Tratamento: 
        //se a posicao do robo em X for MAIOR que 0 
        if(x[0] > 0){
            //se x[2] for maior que x[8]
            if(x[3] > x[9]){
                 valpos = x[3] - x[9];       //distância
            
            if(valpos > 0.015){              //se distância maior que 0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-] MOVIMENTACAO DETECTADA PELO SENSOR S7 [-][-][-][-][-][-]\n");
               contled = contled + 2500;     //flag para acender o LED
            }               
          }   
          else{
            //se x[8] for maior que x[2]
            valpos = x[9] - x[3];           //distância
            if(valpos > 0.015){             //se distância maior que 0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-] MOVIMENTACAO DETECTADA PELO SENSOR S7 [-][-][-][-][-][-]\n");
               contled = contled + 2500;    //flag para acender o LED
            }
          }  
        }
        //Tratamento: 
        //se a posicao do robo em X for MENOR que 0 
        else if(x[0] < 0){ 
            //se x[2] for maior que x[8]
             if(x[3] > x[9]){
                valneg = (x[9]) - (x[3]);        //distância
            if(valneg < -0.015){                 //se distância menor que -0.01 entao o objeto foi movimentado
              printf("[-][-][-][-][-][-]  MOVIMENTACAO DETECTADA PELO SENSOR S7 [-][-][-][-][-][-] \n");
               contled = contled + 2500;         //flag para acender o LED
            } 
          }
          else{
            valneg = (x[3]) - (x[9]);           //distância
            if(valneg < -0.015){                //se distância menor que -0.01 entao o objeto foi movimentadou
              printf("[-][-][-][-][-][-]  MOVIMENTACAO DETECTADA PELO SENSOR S7 [-][-][-][-][-][-] \n");
              contled = contled + 2500;         //flag para acender o LED
            }
          }
        }
      }
      
      vetContr++;      //Itera o controlador de armazenamento nos vetores x e z
      moreCol++;     //Itera a flag de aumento de colisao
 
     }
      
      
         

      //Aciona os motores(linha reta ou rotacao para esqueda) caso o valor dos sensores forem maior do que o definido
     else if(LeituraSensorProx[1]>500) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
    
     else if(LeituraSensorProx[2]>1000) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[3]>1000) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[4]>1000) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[5]>1000) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[6]>500) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
     }
     
    //linha reta caso sensores nao forem menor que o valor definido
     else {  
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
      vetContr=0;      //Zera o controlador de armazenamento nos vetores x e z
      moreCol=0;      //Zera a flag de aumento de colisao
     }
      
 
    
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); //Supervisor

    wb_motor_set_velocity(MotorEsquerdo,6*AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6*AceleradorDireito);
    
   

   //Tratamento de LEDS:
   //Caso os sensores tenham detectado colisoes com movimentacao suficiente para somar 5000 (3x)
   if(contled > 5000){
      
     //Apaga todos LED
       wb_led_set(Leds[0], 0);
       wb_led_set(Leds[1], 0);
       wb_led_set(Leds[2], 0);
       wb_led_set(Leds[3], 0);
       wb_led_set(Leds[4], 0);
       wb_led_set(Leds[5], 0);
       wb_led_set(Leds[6], 0);
       wb_led_set(Leds[7], 0);
       wb_led_set(Leds[8], 0);
       wb_led_set(Leds[9], 0);
     
     //Pisca LED 3 vezes
     for(int jk = 0; jk <= 3; jk++){
         //acende e apaga o LED  
       wb_led_set(Leds[0], 1);
       wb_led_set(Leds[1], 1);
       wb_led_set(Leds[2], 1);
       wb_led_set(Leds[3], 1);
       wb_led_set(Leds[4], 1);
       wb_led_set(Leds[5], 1);
       wb_led_set(Leds[6], 1);
       wb_led_set(Leds[7], 1);
       wb_led_set(Leds[8], 1);
       wb_led_set(Leds[9], 1);
       wb_led_set(Leds[0], 0);
       wb_led_set(Leds[1], 0);
       wb_led_set(Leds[2], 0);
       wb_led_set(Leds[3], 0);
       wb_led_set(Leds[4], 0);
       wb_led_set(Leds[5], 0);
       wb_led_set(Leds[6], 0);
       wb_led_set(Leds[7], 0);
       wb_led_set(Leds[8], 0);
       wb_led_set(Leds[9], 0);
       
     }
   
   //If contador de colisoes com movimentação for == 0 (primeira colisao)
     if(ledac == 0){
         //acende LED 1
          wb_led_set(Leds[0], 1);
          
          
     }
     //caso 2
     if(ledac == 1){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
        
          
     }
     //caso 3
     if(ledac == 2){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
         
                 
     }
     //caso 4
     if(ledac == 3){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          
     }
     //caso 5
     if(ledac == 4){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
       
     }
     //caso 6
     if(ledac == 5){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
       
     }
     //caso 7
     if(ledac == 6){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
          wb_led_set(Leds[6], 1);
        
     }
     //caso 8
     if(ledac == 7){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
          wb_led_set(Leds[6], 1);
          wb_led_set(Leds[7], 1);
          
     }
    //caso 9
    if(ledac == 8){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
          wb_led_set(Leds[6], 1);
          wb_led_set(Leds[7], 1);
          wb_led_set(Leds[8], 1);
          
     }
    //caso 10
    if(ledac == 9){
          wb_led_set(Leds[0], 1);
          wb_led_set(Leds[1], 1);
          wb_led_set(Leds[2], 1);
          wb_led_set(Leds[3], 1);
          wb_led_set(Leds[4], 1);
          wb_led_set(Leds[5], 1);
          wb_led_set(Leds[6], 1);
          wb_led_set(Leds[7], 1);
          wb_led_set(Leds[8], 1);
          wb_led_set(Leds[9], 1);
          
     }

     ledac++;           //contador de colisoes com movimentacao ++
     contled = 0;       //Zera flag de colisao com movimentacao, deixando pronto para novas colisoes com movimentacao
     
   }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}



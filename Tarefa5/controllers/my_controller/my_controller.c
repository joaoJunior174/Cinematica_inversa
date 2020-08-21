#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64

void delay (int time_milisec) {
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft = 0.00;
  while (Timeleft < timeValue)
  {
    currentTime = wb_robot_get_time();
    Timeleft = currentTime - initTime;
    wb_robot_step(TIME_STEP);
  }
}

//essa função é responsavel por tornar os angulos negativos em positivos
double mudarSinal(double Ox){

   if(Ox < 0){
     
      return Ox+2*M_PI;
   
   }
    return Ox;
   
}
//essa função é responsavel por tornar os angulos positivos em negativos
double mudarSinalInverso(double Ox){

   if(Ox > 0){
     
      return Ox-2*M_PI;
   
   }
    return Ox;
   
}
//aqui corrige o problema de que se um angulo for maior que os limites fixos dos angulos de cada eixo do robo, seta para o valor maximo ou minimo
double ajustarAngulos(double Ox, double min, double max){

  if(Ox < min){
     return min;
  }if(Ox > max){
  
    return max;
  }
  
  return Ox;

}
//aqui realiza os mesmos calculos no primeiro video
void calcularCinematica(double x, double y, double z, double theta[3]){
    double l1 = 0.5;
    double l2 = 2;
    double l3 = 2;//valores de l randomicos, melhores selecionados para essa simulação
    double R1 = sqrt(pow(x,2) + pow(y,2));
    double R2 = z-l1;//aqui
    double R3 =  sqrt(pow(R1,2) + pow(R2,2));
    double O1 = atan(y/x);//aqui

    double aux =  (pow(l3,2) - pow(l2,2) - pow(R3,2)) / (-2*l2*R3);
    double aux2 =  (pow(R3,2) - pow(l2,2) - pow(l3,2)) / (-2*l2*l3);
    
    aux = ajustarAngulos(aux,-1,1);//aqui faz o ajuste para cada angulo, pois senão gera muito ruido
    aux2 = ajustarAngulos(aux2,-1,1);
    
    double O2 = atan(R2/R1) - acos(aux);
    double O3 = M_PI - acos(aux2);
    
    O1 = ajustarAngulos(mudarSinal(O1),0,6.03);
    O2 = ajustarAngulos(mudarSinalInverso(O2),-2.44,0);//isso foi necessaio pios as coordenadas do webots é invertida
    O3 = ajustarAngulos(mudarSinal(O3),0.0,4.21);
    
    theta[0]=O1;
    theta[1]=O2;
    theta[2]=O3;
}

int main(int argc, char **argv) {
  
  wb_robot_init();
  
  //a posição de cada uma das bolinhas no mundo
  double posicao_final1[3] = {-0.08, 0.38, 0.27};
  double posicao_final2[3] = {-0.08, 0.19,-0.34};
  double posicao_final3[3] = {0.3, 0.36, 0.0};
  double theta[3];
  
  //aqui se pega os motores
  WbDeviceTag motors[3];
  motors[0] = wb_robot_get_device("base");
  motors[1] = wb_robot_get_device("upperarm");
  motors[2] = wb_robot_get_device("forearm");

  
  //seta a velocidade de cada um dos motores
  for (int i = 0; i < 3; i++)
    wb_motor_set_velocity(motors[i], 1.0);
  
  //para cada uma das bolinhas, calcula os angulos theta para se chegar ate la
  calcularCinematica(posicao_final3[0], posicao_final3[1], posicao_final3[2], theta);
  wb_motor_set_position(motors[0], theta[0]);
  wb_motor_set_position(motors[1], theta[1]);
  wb_motor_set_position(motors[2], theta[2]);
  delay(3000);
  calcularCinematica(posicao_final2[0], posicao_final2[1], posicao_final2[2], theta);
  wb_motor_set_position(motors[0], theta[0]);
  wb_motor_set_position(motors[1], theta[1]);
  wb_motor_set_position(motors[2], theta[2]);
  delay(3000);
  calcularCinematica(posicao_final1[0], posicao_final1[1], posicao_final1[2], theta);
  wb_motor_set_position(motors[0], theta[0]);
  wb_motor_set_position(motors[1], theta[1]);
  wb_motor_set_position(motors[2], theta[2]); 
  delay(3000);

  wb_robot_cleanup();

  return 0;
}

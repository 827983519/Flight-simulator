float getAlpha(int *i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[*i];
   while(n<15){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = a*cos(th)*cos(beta[*i]) + Bi[0][*i];
      q[1] = a*cos(th)*sin(beta[*i]) + Bi[1][*i];
      q[2] = a*sin(th);
      dl[0] = qi[0][*i] - q[0];
      dl[1] = qi[1][*i] - q[1];
      dl[2] = qi[2][*i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);
      if(abs(S-dl2)<0.5){
         return th;
      }
      if(dl2<S){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
}
void getmatrix(float pe[])
{
  float z=pe[5];
  float y=pe[4];
  float x=pe[3];
  M[0][0] = cos(z)*cos(y);
  M[1][0] = sin(z)*cos(y);
  M[2][0] = -sin(y);
  M[0][1] = -sin(z)*cos(x)+cos(z)*sin(x)*sin(y);
  M[1][1] = cos(z)*cos(x)+sin(x)*sin(y)*sin(z);
  M[2][1] = cos(y)*sin(x);
  M[0][2] = sin(z)*sin(x)+cos(z)*sin(y)*cos(x);
  M[1][2] = -cos(z)*sin(y)+sin(z)*sin(y)*cos(x);
  M[2][2] = cos(y)*cos(x);
   }
void getqi(float pe[])
{
   for(int i=0;i<6;i++){
     qi[0][i] = T[0]+M[0][0]*(Pi[0][i])+M[0][1]*(Pi[1][i])+M[0][2]*(Pi[2][i]);
     qi[1][i] = T[1]+M[1][0]*(Pi[0][i])+M[1][1]*(Pi[1][i])+M[1][2]*(Pi[2][i]);
     qi[2][i] = T[2]+M[2][0]*(Pi[0][i])+M[2][1]*(Pi[1][i])+M[2][2]*(Pi[2][i]);
   }
}
void getT(float pe[])
{
   T[0] = pe[0]+H[0];
   T[1] = pe[1]+H[1];
   T[2] = pe[2]+H[2];
}
 void setPos(float pe[]){
    for(int i = 0; i < 6; i++)
    {
        getT(pe);
        getmatrix(pe);
        getqi(pe);//获得了Qi
        theta_a[i]=getAlpha(&i);
        if(i==INV1||i==INV2||i==INV3){
           servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, MIN,MAX);
        }
        else{
           servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, MIN,MAX);
        }
    }
    for(int i = 0; i < 6; i++)
    {
         servo[i].writeMicroseconds(servo_pos[i]);
    }}

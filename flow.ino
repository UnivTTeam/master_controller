#include "flow.h"
#include "device.h"

int hd = 0;
int16_t x_value16bit;
int16_t y_value16bit;

byte x_buffer[2];
byte y_buffer[2];
int data1[9];

//PMW3901からの情報取得　xの差分：x_value16bit、yの差分：y_value16bit
void get_flow(){//なんかシリアル通信の形式が面倒で、なんか色々コードを足してたら何故か読めている関数、変更厳禁！
  while(Serial2.available()){
      byte inChar = (byte)Serial2.read();
      if(inChar == 0xFE && hd == 0){
        hd = 1;
        data1[0] = int(inChar);
      }
      else if(hd == 1){
        if(inChar != 0x04){
          hd =0;
          for( int i=0;i<9;i++){
            data1[i] = 0;
          }
        }
        else{
          hd = hd +1;
          data1[1] = int(inChar);
        }
      }
      else if(hd == 0 && inChar != 0xFE){
        hd = 0;
        for( int i=0;i<9;i++){
            data1[i] = 0;
        }
      }
      else{
          hd = hd +1;
          data1[hd-1] = int(inChar);
      }

      if(hd == 3){//xdiffの取得と計算
        x_buffer[0] = inChar;
      }
      else if( hd == 4 ){
        x_buffer[1] = inChar;
        x_value16bit = x_buffer[1] << 8 | x_buffer[0];
      }

      if(hd == 5){//ydiffの取得と計算
        y_buffer[0] = inChar;
      }
      else if( hd == 6 ){
        y_buffer[1] = inChar;
        y_value16bit = y_buffer[1] << 8 | y_buffer[0];
      }
      else{
      }
      
      if(inChar == 0xAA && hd == 9){
        for( int i=0;i<9;i++){
            data1[i] = 0;
        }
        hd = 0;
      }
      else if (hd >= 9){
        hd = 0;
        for( int i=0;i<9;i++){
            data1[i] = 0;
        }
      }
   }
}
//情報取得終わり

void updateOpticalFlowVelocity()
{
  SensorValue::optical_flow_vx = - y_value16bit * 15 / 18;//* pmw_speed_x_er;//cm/sに換算,値*高度/定数//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  SensorValue::optical_flow_vy = x_value16bit * 15 / 18;// * pmw_speed_x_er;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}
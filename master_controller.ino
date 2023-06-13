
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Serial2.begin(19200, SERIAL_8N1, 19, 18);//オプティカルフロー用シリアル通信

}

int hd = 0;
int16_t x_value16bit;
int16_t y_value16bit;

int16_t tmp;

byte x_buffer[2];
byte y_buffer[2];
int data1[9];

unsigned long tmp_pos_time = 0;//posxyの観測時時間

float pos_x = 0;//x自己位置
float pos_y = 0;//y自己位置
float pos_yaw = 0;//yaw角


float pmw3901_speed_x = 0;//x観測速度
float pmw3901_speed_y = 0;//y観測速度

float pos_x_diff_pmw3901 = 0;//オプティカルフローで観測されたx y の速度(yaw角反映前)
float pos_y_diff_pmw3901 = 0;


void loop() {
  // put your main code here, to run repeatedly:
  get_flow();
  if(millis() - tmp_pos_time > 20){
    xy_odometry();
    tmp_pos_time = millis();
  }
}


void xy_odometry(){
    pmw3901_speed_x = - y_value16bit * 15 / 18;//* pmw_speed_x_er;//cm/sに換算,値*高度/定数//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    pmw3901_speed_y = x_value16bit * 15 / 18;// * pmw_speed_x_er;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    pos_x_diff_pmw3901 = pmw3901_speed_x * (millis() - tmp_pos_time) / 1000 /37 * 190;
    pos_y_diff_pmw3901 = pmw3901_speed_y * (millis() - tmp_pos_time) / 1000 /37 * 190;

    pos_x = pos_x + pos_x_diff_pmw3901;
    pos_y = pos_y + pos_y_diff_pmw3901;
    Serial.print("x:");
    Serial.println(pos_y);//cmマシン座標系で+y
    Serial.print("y:");
    Serial.println(pos_yx);//cmマシン座標系で+x
}

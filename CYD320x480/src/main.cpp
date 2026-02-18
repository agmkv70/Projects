#include <lvgl.h>
#include <TFT_eSPI.h>

#include <ui/ui.h>

#include "touchpad.h"

/*LVGL: Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data ){
  uint16_t touchX, touchY;

  //bool touched = tft.getTouch( &touchX, &touchY, 600 );
  GT911_Scan();
  if ( !touched )
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    /*Set the coordinates*/
    data->point.x = Dev_Now.X[0];
    data->point.y = Dev_Now.Y[0];
    //Serial.printf("touch:%d, x_in:%d, y_in:%d, x_out:%d, y_out:%d\r\n", touched, Dev_Now.X[0], Dev_Now.Y[0], data->point.x, data->point.y);
    data->state = LV_INDEV_STATE_PR;
  }
}

//////////////////////////// DISPLAY ///////////////////////////////
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 480;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 6];
//static lv_color_t buf[ screenWidth * 10 ];

TFT_eSPI tft = TFT_eSPI();
/*LVGL: flush to display*/
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  tft.endWrite();

  lv_disp_flush_ready( disp );
}

/// Setup /////////////////////////////////////////////////
void setup(){ 
  Serial.begin( 115200 ); 
  delay(100);

  GT911_Init_Touchpad();

  lv_init();

  //turn ON backlight:
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL,TFT_BACKLIGHT_ON);

  tft.begin();          
  tft.setRotation(0);   
  //tft.fillScreen(TFT_RED);
  //tft.drawRect(0, 0, 320, 480, TFT_RED);
  //touch_calibrate();
  //uint16_t calData[5] = { 145, 3788, 271, 3535, 1 };
  //uint16_t calData[5] = { 241, 3532, 171, 3685, 3  };
  //tft.setTouch( calData );
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 6 );

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );
  
  //lv_obj_t *label = lv_label_create( lv_scr_act() );
  //lv_label_set_text( label, LVGL_Arduino.c_str() );
  //lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
  //lv_example_btn();
  
  ui_init();

  //#include "user_interface.h"
  //wifi sleep - removed
  Serial.println( "Setup done" );
}

void loop(){
  //GT911_Scan();
  //tft.fillCircle(Dev_Now.X[0], Dev_Now.Y[0], 0, TFT_RED);
  //tft.fillCircle(30, 20, 1, TFT_RED);
  lv_timer_handler(); 
  //tft.fillCircle(Dev_Now.X[0], Dev_Now.Y[0], 1, TFT_RED);
  delay( 10 );


}

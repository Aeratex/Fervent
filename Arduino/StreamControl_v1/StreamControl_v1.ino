
#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define COMPUTE_LOOP_PERIODE 50
#define DISPLAY_LOOP_PERIODE 500
#define DISPLAY_CLEAR_LOOPNB 20
#define ALARM_DISPLAY_DURATION 2000

#define INSP_PRES_SENSOR_PIN A0
#define INSP_FLOW_SENSOR_PIN A2
#define INSP_PRES_SENSOR_OSCIL_PIN 3
#define INSP_VALVE_SERVO_PIN 10
#define EXP_VALVE_SERVO_PIN 11
#define BLOWER_SERVO_PIN 12
#define BLOWER_ANALOG_PIN 6
#define SPEAKER_PIN 5
#define KP_BLOW_STBY A7
#define KP_CANCEL A6
#define KP_ENTER 2
#define KP_UP 8
#define KP_DOWN 9

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/* Controller Objects */
Servo inspValveServo;  // create servo object to control a servo
Servo expValveServo;  // create servo object to control a servo
Servo blowerServo;  // create servo object to control a servo

/* Constant Lookup tables */
const int insp_pres_lookup_raw[] =        {40,    53,  60,  63,   67,   70,   75,   79,   84};
const int insp_pres_lookup_presure[] =  {-10.0, 0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0};
const unsigned char insp_pres_lookup_nbval = 8;
int insp_pres_lookup_raw_measure_offset = -2;

const int insp_flow_lookup_raw[] =        {110, 112, 115, 121, 125, 130, 135, 140};
const int insp_flow_lookup_flow[] =     {0  , 18,   24,  30,  36,  45,  60, 72};   /* In mL/s */
const char insp_flow_lookup_nbval = 8;
int insp_flow_lookup_raw_measure_offset = 0;

const int blower_flow_lookup_pwm[] =       {0, 100, 120, 140, 165, 185, 200, 220};
const int blower_flow_lookup_flow[] =        {0, 100, 300, 400, 500, 600, 750, 1000}; /* mL/s */
const unsigned char blower_flow_lookup_nbval = 8;

const int blower_pres_lookup_pwm[] =       {0, 120, 140, 155, 165, 185, 200,220};
const int blower_pres_lookup_pres[] =        {0, 2 , 5,   7,   10,  15,  20, 30}; /* mmH2O*/
const unsigned char blower_pres_lookup_nbval = 8;

/* Servo and control config value */
const unsigned char insp_valve_open = 165;
const unsigned char exp_valve_open = 160 ;
const unsigned char insp_valve_closed =126 ;
const unsigned char exp_valve_closed = 128;
const unsigned char blower_analog_min = 60;
const unsigned char blower_analog_max= 220;
const unsigned char blower_analog_off = 0;

/* KeyPad Management */
enum KeypadKeysEnum
{
  NONE,
  UP,
  DOWN,
  ENTER,
  CANCEL,
  BLOW_STBY,
};
KeypadKeysEnum keypad_pressed = NONE;

enum SoundsEnum
{
  SILENCE,
  KEYPRESS,
  VALID,
  ERRORS,
};
SoundsEnum sound_request =  SILENCE;

enum VentilModeEnum
{
  CPAP,
  PC,
  VC,
};
VentilModeEnum ventil_mode = VC;

enum MenuItemEnum
{
  HOME,
  MODE,
  IERATIO,
  FREQUENCY,
  PEEP,
  PEAK_PRESURE,
  TIDAL_VOLUME,
  SENSORS_CALIB,
  CPAP_PRESURE,
  ASSIST_PRES,
  ALARM_MAX_VOLUME,
  ALARM_MIN_VOLUME,
  ALARM_HIGH_PRES,
};
MenuItemEnum current_menu = HOME;
byte isModifyingParameter = 0;
int menu_tentative_value = 0;

enum AirwayControlModeEnum
{
  CLOSE,
  OPEN,
  INSP,
  EXP,
};


enum RespStepEnum
{
  INSPIRATION,
  PLATEAU,
  EXPIRATION,
  MAINTAIN,
};
RespStepEnum current_resp_step = INSPIRATION;



/* Air Monitoring variables */
unsigned int insp_pres_raw = 0;
unsigned int insp_pres_raw_max = 0;
float insp_pres_raw_filtered = 0;
float insp_pres_filtered = 0;
unsigned int insp_flow_raw = 0;
unsigned int insp_flow_raw_max = 0;
float insp_flow_raw_filtered = 0;
float insp_flow_filtered = 0;
float insp_volume = 0; /* current inspiratory volumed delivered (mL) */
float max_pres_in_cycle = 0; /* Maximum pressure achieved during the cycle */
float min_pres_in_cycle = 0; /* Maximum pressure achieved during the cycle */

/* Presure PI controller */
float presure_control_error_accu =0;

/* Air Control Parameters */
int pc_peak_pres = 20;  
int cpap_pres = 8;  
int peep_pres = 6;    /* cmH2O */
unsigned int fr = 15;            /* resp per minutes*/
unsigned int vc_tv = 450;        /* Tidal volume mL */
unsigned int ie_ratio = 2;             /* I/E ratio where the number is the Expiratory duration over inpiratory */
unsigned int assist_threshold_pres = 0; /* presure under PEEP Value at which a new breath cycle will be initiated, if 0, then no assist mode */

/* Air Control Alarms parameters */
unsigned int alarm_max_tv_param = 750;  /* Max tidal volume that should not be reached */
unsigned int alarm_min_tv_param = 50; /* Min tidal volume that should be reached */
unsigned int alarm_high_pres_param = 27;     /* Maximum presure that should never been exceeded */

/* Alarms and warnings status  */
unsigned char alarm_pc_peak_not_achieved = 0;
unsigned char alarm_vc_volume_not_achieved = 0;
unsigned char alarm_peep_not_achieved = 0;
unsigned char alarm_min_tv = 0;
unsigned char alarm_max_tv = 0;
unsigned char alarm_high_pres= 0;
unsigned char alarm_low_pres = 0;

/* Intermediate computation variables */
unsigned int set_resp_cycle_time = 0; /* target respiratory cycle time in ms*/
unsigned int time_in_resp_cycle = 0;  /*  Current time in the respiratory cycle in ms */
unsigned long last_resp_time = 0;

float vc_desired_flow =0; /* in mL/s */

/* Status Variables */
unsigned char blow_active = 0;
unsigned char inps_valve_status_open = 0;
unsigned char exp_valve_status_open = 0;
unsigned char add_blower_pwm = 0;
unsigned char display_parameter_page = 0;
unsigned char display_measure_page = 0;
unsigned char turbine_warm_up = 0;
long breath_cycles = 0;
unsigned int blower_pwm = 0;



/* Program Execution variable */
long last_compute_loop_time = 0;
long last_display_loop_time = 0;
long last_alarm_display_time = 0;
long current_time = 0;

/* Display variables */
unsigned char display_loop_ctr = 0; 
unsigned char current_alarm_disp_index = 0;
unsigned char prev_alarm_disp_index = 0;


float Lookup (float raw_value, const int* x_points, const int* y_points, const char nb_value)
{
  float y_result = 0;
  if (raw_value <= x_points[0])
  {
    y_result = float (y_points [0]);
  }
  else if (raw_value >= x_points[nb_value-1])
  {
     y_result = float (y_points [nb_value-1]);
  }
  else
  {
    const char nb_steps = 3; //log ( nb_value)/log (2);
    unsigned int up_index = nb_value;
    unsigned int down_index = 0;
    for (char i = 1; i <= nb_steps; i++)
    {
      if (raw_value >= x_points [ (up_index + down_index)/2])
        down_index = (up_index + down_index)/2;
      else
        up_index = (up_index + down_index)/2;
    }

    y_result =  (raw_value - float(x_points[down_index]))  *  float(y_points[up_index] - y_points[down_index]) / float((x_points[up_index]) - x_points[down_index]) + float(y_points[down_index]);
      /*lcd.setCursor(0,3);
      lcd.print (x_points[down_index]) ;// / float(x_points[up_index] - x_points[down_index]));
      lcd.setCursor(10,3);
      lcd.print (x_points[up_index]) ;// / float(x_points[up_index] - x_points[down_index]));

      */
  }
  

  return y_result;
}

 

void UpdateDisplay ()
{
      display_loop_ctr++;

      if (display_loop_ctr >= DISPLAY_CLEAR_LOOPNB)
      {
        lcd.clear();
       // lcd.init();
        display_loop_ctr = 0;
      }

      
      switch (current_menu)
      {
        case HOME:
          /* -------------------- Upper Mode & Alarms ------------------------------ */
          lcd.setCursor(0,0);
          switch (ventil_mode)
          {
            case CPAP:
              lcd.print("CPAP");
              break;
            case PC:
                  if (assist_threshold_pres > 0)
                      lcd.print("PCVa");
                  else
                      lcd.print("PCV");
                  break;
              break;
            case VC:
                  if (assist_threshold_pres > 0)
                      lcd.print("VCVa");
                  else
                      lcd.print("VCV");
                  break;
              break;
          }

          /* Alarm Display */
         
          if (blow_active && (turbine_warm_up==0))
          {
            char total_alarm = alarm_pc_peak_not_achieved + alarm_vc_volume_not_achieved + alarm_high_pres + alarm_max_tv + alarm_min_tv;
            char alarm_displayed = 0;

            

            /* Alarms Display */
            lcd.setCursor(5,0);

            /* clear alamrm dispay */
            if (current_alarm_disp_index != prev_alarm_disp_index)
            {
              lcd.print("               ");
              lcd.setCursor(5,0);
            }
            prev_alarm_disp_index = current_alarm_disp_index;
            
            switch (ventil_mode)
            {
             case PC:
                if (alarm_pc_peak_not_achieved) {
                   if (current_alarm_disp_index == 0) {
                      lcd.setCursor(5,0);
                      lcd.print("PEAK NOT ACHIEV");
                      if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                      }

                   }
                }
                
              break;
             case VC:
                if (alarm_vc_volume_not_achieved) {
                   if (current_alarm_disp_index == 0)
                   {
                      lcd.setCursor(5,0);
                      lcd.print("Vt NOT ACHIEVE");
                      if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                      }

                   }
                }
                
             break;
           }
           alarm_displayed += (alarm_pc_peak_not_achieved + alarm_vc_volume_not_achieved);

           if (alarm_peep_not_achieved)
           {
              if (current_alarm_disp_index == alarm_displayed)
              {
                      lcd.setCursor(5,0);
                      lcd.print("PEEP NOT ACHIEV");
                      if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                      }
              }
           }
           alarm_displayed += alarm_peep_not_achieved;
                      
           
           if (alarm_high_pres)
           {
              if (current_alarm_disp_index == alarm_displayed)
              {
                      lcd.setCursor(5,0);
                      lcd.print("PRES TOO HIGH");
                      if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                      }
              }
           }
           alarm_displayed += alarm_high_pres;

           if (alarm_min_tv)
           {
              if (current_alarm_disp_index == alarm_displayed)
              {
                    lcd.setCursor(5,0);
                    lcd.print("AIR BLOACKAGE");
                    if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                    }
              }
           }
           alarm_displayed += alarm_min_tv;

           if (alarm_max_tv)
           {
              if (current_alarm_disp_index == alarm_displayed)
              {
                    lcd.setCursor(5,0);
                    lcd.print("AIR LEAK HI V");
                    if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                    }
              }
           }
           alarm_displayed += alarm_max_tv;

           if (alarm_low_pres)
           {
              if (current_alarm_disp_index == alarm_displayed)
              {
                  lcd.setCursor(5,0);
                  lcd.print("AIR LEAK LO P");
                  if (current_time > (last_alarm_display_time + ALARM_DISPLAY_DURATION)) {
                          current_alarm_disp_index ++;
                          last_alarm_display_time = current_time;
                     
                  }
              }
           }
           alarm_displayed += alarm_low_pres;

           if (current_alarm_disp_index >= total_alarm)
                  current_alarm_disp_index = 0;

            
          }
          else
          {
             if (turbine_warm_up == 0)
             {
              lcd.setCursor(15,0);
              lcd.print("stby");
             }
             else
             {
              lcd.setCursor(12,0);
              lcd.print("warm up");
             }
          }
          

          /* Ventilation Parameters */
         lcd.setCursor(1,1);
         switch (display_parameter_page)
         {
          case 0:
              switch (ventil_mode)
              {
                 case PC:
                   lcd.print("Ppeak=");
                   lcd.setCursor(8,1);
                   lcd.print(pc_peak_pres);
                   break;
                 case VC:
                   lcd.print("Vt=");
                   lcd.setCursor(5,1);
                   lcd.print(vc_tv);
                   break;
                  case CPAP:
                   lcd.print("Pres=");
                   lcd.setCursor(8,1);
                   lcd.print(cpap_pres);
                   break;
              }
             if (ventil_mode != CPAP)
             {
               lcd.setCursor(12,1);
               lcd.print("PEEP=");
               lcd.setCursor(18,1);
               lcd.print(peep_pres);
               lcd.setCursor(1,2);             
               lcd.print("Fr=");
               lcd.setCursor(5,2);
               lcd.print(fr);
               lcd.setCursor(12,2);
               lcd.print("I/E= 1/");
               lcd.setCursor(19,2);
               lcd.print(ie_ratio);
             }
             break;
          case 1:
               
                lcd.setCursor(0,1);
                lcd.print("Blow=");
                lcd.setCursor(6,1);
                lcd.print(blower_pwm);
               switch (ventil_mode)
              {
                 case PC:
                  
                   break;
                 case VC:
                   lcd.setCursor(0,2);
                   lcd.print("Fset=");
                   lcd.setCursor(6,2);
                   lcd.print(vc_desired_flow);
                   break;
                   
                  case CPAP:
                    
                   break;
              } 
              
          
          break;
        }
        break;


    /* ------------------- MENU ------------------ */
        case MODE:
          lcd.setCursor(1,0);
          lcd.print("MENU > MODE");
          lcd.setCursor(0,1);
           if (isModifyingParameter)
               lcd.print("> Mode:");
           else
               lcd.print("Mode:");
  
            lcd.setCursor(12,1);
            lcd.print("    ");
            lcd.setCursor(12,1);
            if (isModifyingParameter){
              switch (menu_tentative_value)
              {
                case 0:
                  if (assist_threshold_pres > 0)
                      lcd.print("VCVa");
                  else
                      lcd.print("VCV");
                  break;
                case 1:
                   if (assist_threshold_pres > 0)
                      lcd.print("PCVa");
                   else
                      lcd.print("PCV");
                  break;
                case 2:
                  lcd.print("CPAP");
                  break;
              }
            }
            else {
              switch (ventil_mode)
              {
                case CPAP:
                  lcd.print("CPAP");
                  break;
                case PC:
                  lcd.print("PCV");
                  break;
                case VC:
                  lcd.print("VCV");
                  break;
              }
           }
        
        break;
        

       case PEAK_PRESURE:
          lcd.setCursor(1,0);
          lcd.print("MENU > PEAK PRES");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Peak presure=");
          else
            lcd.print("Peak presure=");
          
          lcd.setCursor(16,1);
          lcd.print("    ");
          lcd.setCursor(16,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(pc_peak_pres);
        break;

        
       case TIDAL_VOLUME:
          lcd.setCursor(1,0);
          lcd.print("MENU > TIDAL VOL");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Volume=");
          else
            lcd.print("Volume=");
          
          lcd.setCursor(16,1);
          lcd.print("    ");
          lcd.setCursor(16,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(vc_tv);
        break;

       case CPAP_PRESURE:
          lcd.setCursor(1,0);
          lcd.print("MENU > CPAP PRES");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Presure=");
          else
            lcd.print("Presure=");
          
          lcd.setCursor(16,1);
          lcd.print("    ");
          lcd.setCursor(16,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(cpap_pres);
        break;

       case PEEP:
          lcd.setCursor(1,0);
          lcd.print("MENU > PEEP PRES");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> PEEP presure=");
          else
            lcd.print("PEEP presure=");
          
          lcd.setCursor(16,1);
          lcd.print("    ");
          lcd.setCursor(16,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(peep_pres); 
        break;

        case IERATIO:
          lcd.setCursor(1,0);
          lcd.print("MENU > I/E RATIO");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> I/E ratio=1/");
          else
            lcd.print("I/E ratio=  1/");
          
          lcd.setCursor(14,1);
          lcd.print("    ");
          lcd.setCursor(14,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(ie_ratio); 
        break;

       case FREQUENCY:
          lcd.setCursor(1,0);
          lcd.print("MENU > RESP FREQ");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Resp Freq=");
          else
            lcd.print("Resp Freq=");
          
          lcd.setCursor(14,1);
          lcd.print("    ");
          lcd.setCursor(14,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(fr); 
        break;

        case ASSIST_PRES:
          lcd.setCursor(1,0);
          lcd.print("MENU > ASSIST");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Pres thld= -");
          else
            lcd.print("Pres thld=   -");
          
          lcd.setCursor(16,1);
          lcd.print("    ");
          lcd.setCursor(16,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(assist_threshold_pres);
        break;

        case ALARM_HIGH_PRES:
          lcd.setCursor(1,0);
          lcd.print("MENU > ALRM HI PRES");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Max Pres=");
          else
            lcd.print("Max Pres=");
          
          lcd.setCursor(14,1);
          lcd.print("    ");
          lcd.setCursor(14,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(alarm_high_pres_param); 
        break;

       case ALARM_MAX_VOLUME:
          lcd.setCursor(1,0);
          lcd.print("MENU > ALRM MAX VOL");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Max Vt=");
          else
            lcd.print("Max Vt=");
          
          lcd.setCursor(14,1);
          lcd.print("    ");
          lcd.setCursor(14,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(alarm_max_tv_param); 
        break;

      case ALARM_MIN_VOLUME:
          lcd.setCursor(1,0);
          lcd.print("MENU > ALRM MIN VOL");
          lcd.setCursor(0,1);
          if (isModifyingParameter)
             lcd.print("> Min Vt=");
          else
            lcd.print("Min Vt=");
          
          lcd.setCursor(14,1);
          lcd.print("    ");
          lcd.setCursor(14,1);
          if (isModifyingParameter)
            lcd.print(menu_tentative_value);
          else
            lcd.print(alarm_min_tv_param); 
        break;
        
        case SENSORS_CALIB:
          lcd.setCursor(1,0);
          lcd.print("MENU > SENSORS CALIB");
          lcd.setCursor(0,2);
          lcd.print("P raw=");
          lcd.setCursor(6,2);
          lcd.print("    ");
          lcd.setCursor(6,2);
          lcd.print(int (insp_pres_raw_filtered) );
          
          lcd.setCursor(10,2);
          lcd.print("F raw=");
          lcd.setCursor(16,2);
          lcd.print("    ");
          lcd.setCursor(16,2);
          lcd.print(int (insp_flow_raw_filtered));
          
          lcd.setCursor(0,1);
          if (isModifyingParameter)
          {
            lcd.print("> Blower=");
            lcd.setCursor(14,1);
            lcd.print("    ");
            lcd.setCursor(14,1);
            lcd.print(menu_tentative_value);
          }
          else
          {
             lcd.setCursor(0,1);
            lcd.print("Blower=");
            lcd.setCursor(14,1);
            lcd.print("    ");
            lcd.setCursor(14,1);
            lcd.print(blower_pwm);
          }
      
          
        break;
      }


      
      /* ------------------- LOWER MEASURE BAR ------------------ */
      
      /* Always display the current status information */
      const int current_info_y_offset = 7;

      /* Presure bar graph */
      lcd.setCursor(0,3);
      if (blow_active)
      {
          switch (current_resp_step)
          {
            case INSPIRATION:
                 lcd.print(" >      ");
            break;
            case PLATEAU:
                 lcd.print(" ^      ");
            break;
            case EXPIRATION:
                 lcd.print(" <      ");
            break;
            case MAINTAIN:
              lcd.print(" +      ");
             break;
          }
      }
       else
       {
            if ( (inps_valve_status_open == 0) && (exp_valve_status_open == 0))
              lcd.print(" x      ");
            if ( (inps_valve_status_open > 0) && (exp_valve_status_open > 0))
              lcd.print(" o      ");
            if ( (inps_valve_status_open == 0) && (exp_valve_status_open > 0))
              lcd.print(" <      ");
            if ( (inps_valve_status_open > 0) && (exp_valve_status_open == 0))
              lcd.print(" >      ");
       }


      if (int (insp_pres_filtered) < 0)
      {
         lcd.setCursor(0,3);
         lcd.print("-");
      } 
      else if  (int (insp_pres_filtered) > 0){ 
        for (unsigned char i = 0; i <= (int (insp_pres_filtered) / 10); i++)
        {
          if (i < 5)
          {
            lcd.setCursor(i + 2,3);
            lcd.print("-");
          }
        }
      }
        
     switch (display_measure_page)
     {
      case 0:
        lcd.setCursor(0 + current_info_y_offset,3);
        lcd.print("P");
        lcd.setCursor(1 + current_info_y_offset,3);
        lcd.print("   ");
        lcd.setCursor(1 + current_info_y_offset,3);
        lcd.print(int (insp_pres_filtered));
  
        lcd.setCursor(4 + current_info_y_offset,3);
        lcd.print("F");
        lcd.setCursor(5 + current_info_y_offset,3);
        lcd.print("   ");
        lcd.setCursor(5 + current_info_y_offset,3);
        lcd.print(int (insp_flow_filtered));  

        if (ventil_mode != CPAP)
        {
          lcd.setCursor(8 + current_info_y_offset,3);
          lcd.print("V");
          lcd.setCursor(9 + current_info_y_offset,3);
          lcd.print("   ");
          lcd.setCursor(9 + current_info_y_offset,3);
          lcd.print(int (insp_volume));  
        }
        break;
      case 1:
        lcd.setCursor(0 + current_info_y_offset,3);
        lcd.print("Pmax");
        lcd.setCursor(4 + current_info_y_offset,3);
        lcd.print("  ");
        lcd.setCursor(4 + current_info_y_offset,3);
        lcd.print(int (max_pres_in_cycle));

        lcd.setCursor(7 + current_info_y_offset,3);
        lcd.print("Pmin");
        lcd.setCursor(11 + current_info_y_offset,3);
        lcd.print("   ");
        lcd.setCursor(11 + current_info_y_offset,3);
        lcd.print(int (min_pres_in_cycle));
        break;
      }

}

void PlaySounds(void)
{
  static SoundsEnum current_sound =   SILENCE;
  static long last_sound_request_time = 0;

  if ( (sound_request != current_sound) && (sound_request!=   SILENCE))
  {
    current_sound = sound_request;
    last_sound_request_time = current_time;
  }
  switch (current_sound)
  {
    case KEYPRESS:
      analogWrite ( SPEAKER_PIN, 100 );
      if ( (current_time - last_sound_request_time) > 150)
        current_sound =   SILENCE;
      break;
    case VALID:
       analogWrite ( SPEAKER_PIN, 100 );
       if ( (current_time - last_sound_request_time) > 500)
        current_sound =   SILENCE;
      break;
    case ERRORS:
       analogWrite ( SPEAKER_PIN, 175 );
       if ( (current_time - last_sound_request_time) > 1500)
        current_sound =   SILENCE;
      break;
    default:
       analogWrite ( SPEAKER_PIN,  0);
  }
  sound_request = SILENCE;
}

void BlowerControl (void)
{
    unsigned int base_pwm = 0;
    //char total_pwm = 0;

    /* Blower Maangement */
    if (!blow_active)
    {
        if ( !( (isModifyingParameter ==1) && (current_menu == SENSORS_CALIB))) /* if we are not in the manual calibration mode */
        {
            analogWrite ( BLOWER_ANALOG_PIN, 0 );

        }
        return;
    }
    else
    {
        switch (ventil_mode)
        {
           case CPAP:
                base_pwm =Lookup ( int (cpap_pres), (const int*) &blower_pres_lookup_pres[0],  (const int*) &blower_pres_lookup_pwm[0], blower_pres_lookup_nbval)*1.2;
                break;
           case PC:
                base_pwm =Lookup ( int (pc_peak_pres), (const int*) &blower_pres_lookup_pres[0],  (const int*) &blower_pres_lookup_pwm[0], blower_pres_lookup_nbval)*1.1;
                break;
           case VC:
                vc_desired_flow = float (vc_tv) / (60.0/ (fr *(1.0 + float(ie_ratio)))); // (1.0/(fr/60.0) * (1.0/(1.0 + float(ie_ratio)))) ; /* in mL/s */ 
                unsigned int base_pwm_flow = Lookup ( int (vc_desired_flow), (const int*) &blower_flow_lookup_flow[0],  (const int*) &blower_flow_lookup_pwm[0], blower_flow_lookup_nbval)*1.3;
                unsigned int base_pwm_peep = Lookup ( int (peep_pres), (const int*) &blower_pres_lookup_pres[0],  (const int*) &blower_pres_lookup_pwm[0], blower_pres_lookup_nbval)*1.2;
                base_pwm = max(base_pwm_flow, base_pwm_peep); 
                break;
           break;
       
        
        
        }
      
       blower_pwm = base_pwm + add_blower_pwm;

       if (blower_pwm > blower_analog_max){
          blower_pwm = blower_analog_max;
       }
       
       if (blower_pwm > 250)
       {
          blower_pwm = 250;
       }

       analogWrite ( BLOWER_ANALOG_PIN, blower_pwm);
    }

}

void RequestMoreBlower (void)
{
    if (add_blower_pwm < 100)
        add_blower_pwm += 5;

}

void RequestLessBlower (void)
{
    if (add_blower_pwm > 0)
        add_blower_pwm -= 2;

}

void AquireKeypad (void)
{
  /* Keyboard Maangement */
  if (analogRead(KP_BLOW_STBY) <100)
    keypad_pressed = BLOW_STBY;
  if (analogRead(KP_CANCEL) <100)
    keypad_pressed = CANCEL;
  if (digitalRead(KP_ENTER) == LOW)
    keypad_pressed = ENTER;
  if (digitalRead(KP_UP) == LOW)
    keypad_pressed = UP;
  if (digitalRead(KP_DOWN) == LOW)
    keypad_pressed = DOWN;
}

void InitVentilation (void)
{
    blow_active = 1;
    turbine_warm_up = 0;
    current_resp_step = INSPIRATION;
    add_blower_pwm = 0;
    breath_cycles = 0;
    alarm_vc_volume_not_achieved = 0;
     alarm_pc_peak_not_achieved = 0;
     alarm_high_pres = 0;
     alarm_max_tv = 0;
     alarm_min_tv = 0;
     alarm_low_pres = 0;
     alarm_peep_not_achieved = 0;
    inspValveServo.attach(INSP_VALVE_SERVO_PIN); 
    expValveServo.attach(EXP_VALVE_SERVO_PIN);
    last_resp_time = current_time;
     
}

void StopVentilation (void)
{
     blow_active = 0;
     alarm_vc_volume_not_achieved = 0;
     alarm_pc_peak_not_achieved = 0;
     alarm_high_pres = 0;
     alarm_max_tv = 0;
     alarm_min_tv = 0;
     alarm_low_pres = 0;
     alarm_peep_not_achieved = 0;
     current_resp_step = INSPIRATION;
     AirwayControl (OPEN, 100);
     turbine_warm_up = 0;
     delay (100);
     inspValveServo.detach(); 
     expValveServo.detach(); 
}

void ProceedKeypad (void)
{
     if (isModifyingParameter ==0)
     {
        menu_tentative_value = 0;
     }
     if (keypad_pressed != NONE)
     { 
        sound_request = KEYPRESS;
        lcd.clear();

         /* Cancel button */
         if ( (keypad_pressed == CANCEL) && (current_menu != HOME) )
         {
                if (isModifyingParameter == 1) {
                  isModifyingParameter = 0;
                  sound_request = ERRORS;
                }
                else {
                  isModifyingParameter = 0;
                  current_menu = HOME;
                }
         }
  
        switch (current_menu)
        {
          case HOME:
            switch (keypad_pressed)
            {
              case BLOW_STBY:
                if (blow_active == 0)
                {
                    InitVentilation();
                }
                else
                {
                    StopVentilation ();
                }
                break;
              case ENTER:
                switch (ventil_mode)
                {
                  case PC:
                    current_menu = PEAK_PRESURE;
                    break;
                  case VC:
                    current_menu = TIDAL_VOLUME;
                    break;
                  case CPAP:
                    current_menu = CPAP_PRESURE;
                    break;
                }
                break;
              case DOWN:
                display_measure_page ++;
                if (display_measure_page > 1)
                    display_measure_page = 0;
                 break;
             case UP:
                display_parameter_page ++;
                if (display_parameter_page > 1)
                    display_parameter_page = 0;
                break;

              case CANCEL:
                lcd.init();
                break;
            }
            break;

          case PEAK_PRESURE:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                  if (menu_tentative_value < pc_peak_pres) /* Reset the additional blower PWM if the new peak presure is lower than the previous one */
                      add_blower_pwm = 0; 
                  pc_peak_pres = menu_tentative_value;
                  isModifyingParameter = 0;
                  sound_request = VALID;
                }
                else
                {
                  isModifyingParameter = 1;
                  menu_tentative_value = pc_peak_pres;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 30 )
                     menu_tentative_value += 2; 
                }
                else {

                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if ( (menu_tentative_value > 10) && (menu_tentative_value > (peep_pres + 5.0))  )
                     menu_tentative_value -= 2; 
                }
                else {
                    current_menu = PEEP;
                }
                break;

          
            }
          break;

         case TIDAL_VOLUME:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                  if (menu_tentative_value < vc_tv) /* Reset the additional blower PWM if the new peak presure is lower than the previous one */
                      add_blower_pwm = 0; 
                  vc_tv = menu_tentative_value;
                  isModifyingParameter = 0;
                  sound_request = VALID;
                }
                else
                {
                  isModifyingParameter = 1;
                  menu_tentative_value = vc_tv;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 600 )
                     menu_tentative_value += 25; 
                }
                else {

                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if ( menu_tentative_value > 150)
                     menu_tentative_value -= 25; 
                }
                else {
                    current_menu = PEEP;
                }
                break;

          
            }
          break;

          case CPAP_PRESURE:
            switch (keypad_pressed)
            {
             case ENTER:
                if (isModifyingParameter == 1)
                {
                  if (menu_tentative_value < pc_peak_pres) /* Reset the additional blower PWM if the new peak presure is lower than the previous one */
                      add_blower_pwm = 0; 
                  cpap_pres = menu_tentative_value;
                  isModifyingParameter = 0;
                  sound_request = VALID;
                }
                else
                {
                  isModifyingParameter = 1;
                  menu_tentative_value = cpap_pres;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 18 )
                     menu_tentative_value += 1; 
                }
                else {

                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if  (menu_tentative_value > 4)
                     menu_tentative_value -= 1; 
                }
                else {
                    current_menu = MODE;
                }
                break;

          
            }
          break;

          case PEEP:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    peep_pres= menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = peep_pres;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if ((menu_tentative_value < 20) && (menu_tentative_value < (pc_peak_pres-5.0)) )
                     menu_tentative_value += 2; 
                }
                else {
                    switch (ventil_mode)
                    {
                      case PC:
                        current_menu = PEAK_PRESURE;
                        break;
                      case VC:
                        current_menu = TIDAL_VOLUME;
                        break;
                      case CPAP:
                        current_menu = CPAP_PRESURE;
                        break;
                    }
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 0 )
                     menu_tentative_value -= 2; 
                }
                else {
                      current_menu = IERATIO;
                }
                break;
              }
          break;

         case IERATIO:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    ie_ratio = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = ie_ratio;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 5 )
                     menu_tentative_value += 1; 
                }
                else {
                  current_menu = PEEP;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 1 )
                     menu_tentative_value -= 1; 
                }
                else {
                      current_menu = FREQUENCY;
                }
                break;
              }
          break;

          case FREQUENCY:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    fr = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = fr;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 40 )
                     menu_tentative_value += 1; 
                }
                else {
                  current_menu = IERATIO;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 5 )
                     menu_tentative_value -= 1; 
                }
                else {
                      current_menu = ASSIST_PRES;
                }
                break;
              }
          break;

          
          case ASSIST_PRES:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    assist_threshold_pres = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = assist_threshold_pres;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 8 )
                     menu_tentative_value += 4; 
                }
                else {
                  current_menu = FREQUENCY;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 0 )
                     menu_tentative_value -= 4; 
                }
                else {
                      current_menu = MODE;
                }
                break;
              }
          break;

         case MODE:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                   // fr = menu_tentative_value;
                    switch (menu_tentative_value)
                    {
                      case 0:
                        ventil_mode = VC;
                        break;
                      case 1:
                        ventil_mode = PC;
                        break;
                      case 2:
                        ventil_mode = CPAP;
                        break;
                    }
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    switch (ventil_mode)
                    {
                    case VC:
                      menu_tentative_value = 0;
                      break;
                    case PC:
                      menu_tentative_value = 1;
                      break;
                    case CPAP:
                      menu_tentative_value = 2;
                      break;
                    }
                }
                break;

              case UP:
                if (isModifyingParameter) {
                    if (menu_tentative_value < 2)
                      menu_tentative_value ++;
                }
                else {
                  if (ventil_mode != CPAP)
                    current_menu = ASSIST_PRES;
                  else
                     current_menu = CPAP_PRESURE;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                   if (menu_tentative_value > 0)
                      menu_tentative_value --;
                }
                else {
                      current_menu = ALARM_MAX_VOLUME;
                }
                break;

            
            }
            break;

         case ALARM_MAX_VOLUME:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    alarm_max_tv_param = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = alarm_max_tv_param;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 750)
                     menu_tentative_value += 25; 
                }
                else {
                  current_menu = MODE;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if ( (menu_tentative_value > 150) && (menu_tentative_value > alarm_min_tv_param + 50) )
                     menu_tentative_value -= 25; 
                }
                else {
                      current_menu = ALARM_MIN_VOLUME;
                }
                break;
              }
          break;

         case ALARM_MIN_VOLUME:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    alarm_min_tv_param = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = alarm_min_tv_param;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if ( (menu_tentative_value < 250) && (menu_tentative_value < alarm_max_tv_param - 50)  )
                     menu_tentative_value += 25; 
                }
                else {
                  current_menu = ALARM_MAX_VOLUME;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 100 )
                     menu_tentative_value -= 25; 
                }
                else {
                      current_menu = ALARM_HIGH_PRES;
                }
                break;
              }
          break;

         case ALARM_HIGH_PRES:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 1)
                {
                    alarm_high_pres_param = menu_tentative_value;
                    isModifyingParameter = 0;
                    sound_request = VALID;
                }
                else
                {
                    isModifyingParameter = 1;
                    menu_tentative_value = alarm_high_pres_param;
                }
                break;
              case UP:
                if (isModifyingParameter) {
                   if (menu_tentative_value < 30 )
                     menu_tentative_value += 2; 
                }
                else {
                  current_menu = ALARM_MIN_VOLUME;
                }
                break;
              case DOWN:
                if (isModifyingParameter) {
                  if (menu_tentative_value > 10 )
                     menu_tentative_value -= 2; 
                }
                else {
                      current_menu = SENSORS_CALIB;
                }
                break;
              }
          break;
         
             
          case SENSORS_CALIB:
            switch (keypad_pressed)
            {
              case ENTER:
                if (isModifyingParameter == 0) {
                  isModifyingParameter = 1;
                      inspValveServo.attach(INSP_VALVE_SERVO_PIN); 
                      expValveServo.attach(EXP_VALVE_SERVO_PIN); 
                }
                else {
                    isModifyingParameter = 0;
                      sound_request = VALID;

                     AirwayControl (OPEN, 100);
                     delay (100);
                     inspValveServo.detach(); 
                     expValveServo.detach(); 
                     analogWrite ( BLOWER_ANALOG_PIN, 0);

                }
                break;
              case UP:
                if (isModifyingParameter)
                {
                  if (menu_tentative_value < blower_analog_max){
                    menu_tentative_value += 5; 
                  }
                }
                else {
                    current_menu = MODE;
                }
                break;
                
              case DOWN:
                if (isModifyingParameter)
                {
                  if (menu_tentative_value > 0)
                  {
                    menu_tentative_value -= 5; 
                  }
                }
                else
                {
                   
                }
                break;

           
            }
            if ( isModifyingParameter == 1)
            {
              AirwayControl (INSP, 100);
              blow_active = 0;
              analogWrite ( BLOWER_ANALOG_PIN, menu_tentative_value);
            }
            else
            {
              
            }
            break;
           
        }

       
      }      
}

void AirwayControl (AirwayControlModeEnum mode, float percentage)
{
  float valve_angle = 0;

  if (percentage < 0)
    percentage = 0;

  if (percentage > 100)
    percentage = 100;

  switch (mode)
  {
    case OPEN:
      inspValveServo.write(insp_valve_open); 
      expValveServo.write(exp_valve_open);
      inps_valve_status_open = 100;
      exp_valve_status_open = 100;


      break;
    case CLOSE:
      inspValveServo.write(insp_valve_closed); 
      expValveServo.write(exp_valve_closed);
      inps_valve_status_open = 0;
      exp_valve_status_open = 0;
      break;
    case INSP:
        expValveServo.write(exp_valve_closed);
        valve_angle = (insp_valve_open - insp_valve_closed) * (float (percentage) / 100.0) + insp_valve_closed; 
        inspValveServo.write(int (valve_angle));
        inps_valve_status_open = percentage;
        exp_valve_status_open = 0;
      break;

    case EXP:
        inspValveServo.write(insp_valve_closed);
        valve_angle = (exp_valve_open - exp_valve_closed) * (float (percentage) / 100.0) + exp_valve_closed; 
        expValveServo.write(int (valve_angle));
        inps_valve_status_open = 0;
        exp_valve_status_open = percentage;
      break;
  }
}

void MaintainPresureVentilation (float presure_set_point, float presure_measure)
{
  float presure_error= (presure_measure - presure_set_point) ;

  if (abs (presure_error) >=1 )
  {
    
   
  }

 //if (ventil_mode != CPAP)
 //{
    if (abs (presure_error) < 0.5)
          presure_control_error_accu = 0;
  //}

  
    presure_control_error_accu += presure_error *0.25;

   if (presure_control_error_accu > 50)
      presure_control_error_accu = 50;
    if (presure_control_error_accu < -50)
      presure_control_error_accu = -50;

  float pi =   presure_error*0.5 + presure_control_error_accu   ;

  //lcd.setCursor(1,1);
  //lcd.print(pi);
  
//         if (abs (pi) < 1.0)
//         {
//            AirwayControl (CLOSE, 100);
//             //AirwayControl (EXP, (14 - presure_set_point) * 3.0 );
//         }
//         else 
         
         if ( pi > 0)

         {
              if (ventil_mode == CPAP)
                    AirwayControl (EXP, 25.0* presure_error);
              else
                   AirwayControl (EXP, 25.0* pi);
         }
         else
         {
              AirwayControl (INSP,  7.5  * abs (pi)  );// presure_set_point*
         }
}

void MaintainFlowVentilation (float volume_set_point, float volume_measure)
{
//   float volume_error= (volume_measure - volume_set_point) ;
//   //presure_control_error_accu += presure_error *0.5;
//
//   if (volume_error > 100)
//      volume_error = 100;
//    if (volume_error < -100)
//      volume_error = -100;
//
//  float pi = volume_error /100.0; // presure_control_error_accu + presure_error*0.5  ;
//
//  //lcd.setCursor(1,1);
//  //lcd.print(pi);
//  
//         if (pi > 0)
//              AirwayControl (EXP, 50* pi + 80);
//         else
//              AirwayControl (INSP, 50* pi + 60  ); 
}

void SerialOutput (void)
{
  //char buf[30];
  //sprintf("%d ",current_time); //, int(insp_pres_filtered), int(insp_flow_filtered), insp_volume , %d , %d, %f
  Serial.print(float (current_time/1000.0));
  Serial.print(",");
  Serial.print(float(insp_pres_filtered));
  Serial.print(",");
  Serial.print(insp_volume);
  Serial.print("\n");
  
}


void UpdateAirMeasure (void)
{
  /* Inspiratory Presure Sensor */
  insp_pres_raw = analogRead(INSP_PRES_SENSOR_PIN) ; //+ insp_pres_lookup_raw_measure_offset; 
  if (insp_pres_raw > insp_pres_raw_max)
    insp_pres_raw_max = insp_pres_raw;
  //insp_pres_raw_filtered += insp_pres_raw;
  insp_pres_raw_filtered = insp_pres_raw_filtered * 0.95 + insp_pres_raw * 0.05;

  /* Inpiratory Flow Sensor */
  insp_flow_raw = analogRead(INSP_FLOW_SENSOR_PIN); // + insp_flow_lookup_raw_measure_offset; 
  if (insp_flow_raw > insp_flow_raw_max)
    insp_flow_raw_max = insp_flow_raw;
  insp_flow_raw_filtered = insp_flow_raw_filtered * 0.95 + insp_flow_raw * 0.05;
  
}


void ManageVentilation (void)
{

    if ( (blow_active == 0))
    {
        if ( !( (isModifyingParameter ==1) && (current_menu == SENSORS_CALIB))) /* if we are not in the manual calibration mode, otherwise air is manager by the manual calib mode */
        {
            AirwayControl (OPEN, 100);
        }
        turbine_warm_up = 0;
        return;
    }
    else
    {
      if ((breath_cycles <= 5) && (ventil_mode != CPAP) ){
        AirwayControl (OPEN, 100);
        turbine_warm_up = 1;
      }
      else
      {
          turbine_warm_up = 0;
      }

    }


    if (ventil_mode == CPAP) /* CPAP does not use breath cycle */
    {
      MaintainPresureVentilation (cpap_pres, insp_pres_filtered);
      if ( (current_time - last_resp_time) > 4000)
      {
        if (max_pres_in_cycle < cpap_pres)
        {
           RequestMoreBlower ();
           
        }
       if (max_pres_in_cycle > (cpap_pres*1.5))
        {
           RequestLessBlower ();
        }
        max_pres_in_cycle = 0;
         last_resp_time = current_time; 
      }
     
      return;
        
   }
    
    
    /* End Last cycle and Sart New cycle */
    if ( blow_active && ( (time_in_resp_cycle > set_resp_cycle_time) ||  ( (current_resp_step == MAINTAIN) && ( insp_pres_filtered <  (peep_pres - assist_threshold_pres) ) && (assist_threshold_pres > 0) ) ))
    {
         /* Diagnosis of the last breath cycle */
         if (breath_cycles > 5 )
         {

            
             if (insp_volume >= alarm_max_tv_param)
                alarm_max_tv = 1;
             else
                alarm_max_tv = 0;

              if (insp_volume < alarm_min_tv_param)
                alarm_min_tv = 1;
             else
                alarm_min_tv = 0;

              if (max_pres_in_cycle >= alarm_high_pres_param)
                  alarm_high_pres = 1;
              else
                  alarm_high_pres = 0;

             if (max_pres_in_cycle < (peep_pres + 2.0) )
             {
                    alarm_low_pres = 1;
                    RequestMoreBlower ();
             }
             else
                   alarm_low_pres = 0;



             
             switch (ventil_mode)
             {
              case PC:
                if ( max_pres_in_cycle < (pc_peak_pres - 3.0)){
                  alarm_pc_peak_not_achieved = 1;
                  RequestMoreBlower ();
                }
                else {
                  alarm_pc_peak_not_achieved = 0;         
                }
              break;
              case VC:
                if ( insp_volume < (vc_tv * 0.85) ){
                  alarm_vc_volume_not_achieved = 1;
                  RequestMoreBlower ();
                }
                else {
                  alarm_vc_volume_not_achieved = 0;         
              } 
    
              
              break;
             }
         }


        /* start new cycle */ 
        last_resp_time = current_time;
        current_resp_step = INSPIRATION;
        insp_volume = 0;
        max_pres_in_cycle = 0;
        min_pres_in_cycle = 50;
        breath_cycles++;
    }
    
   

 
    /* Compute the current step of the respiratory cycle */
    switch (ventil_mode)
    {
      case PC:
      if ( time_in_resp_cycle < (set_resp_cycle_time * 1.0/(1.0+float(ie_ratio) )) )   //float(IEratio)  * 1.0/(1.0+1.0)) 
      { 
          if ((insp_pres_filtered < pc_peak_pres) && (current_resp_step != PLATEAU) && (insp_pres_filtered < alarm_high_pres_param) ){
            current_resp_step = INSPIRATION;
          }
          else if ( (current_resp_step == INSPIRATION) ){
            current_resp_step = PLATEAU;
            if (insp_pres_filtered >= alarm_high_pres_param)
            {
                    alarm_high_pres = 1;
            }
          }
          
      }
      else
      {
          if ( (current_resp_step == INSPIRATION) || (current_resp_step == PLATEAU)) {
            current_resp_step = EXPIRATION;  
          }
          if (( current_resp_step == EXPIRATION) && (insp_pres_filtered < peep_pres + 1.0))
          {
            current_resp_step = MAINTAIN; 
          }
      }

      /* Execture breath action */
      switch (current_resp_step)
      {
        case INSPIRATION:
            //MaintainPresureVentilation (pc_peak_pres, insp_pres_filtered);
            AirwayControl (INSP, 100);
             presure_control_error_accu = 0; 
           break;
        case PLATEAU:
           MaintainPresureVentilation (pc_peak_pres, insp_pres_filtered);
           break;
       case EXPIRATION:
            AirwayControl (EXP, ( insp_pres_filtered - (peep_pres) ) / 4.0 * 100.0 );
              /*
             if ( abs (peep_pres - insp_pres_filtered) > 3.0)
                    AirwayControl (EXP, 100);
              else
                  //AirwayControl (EXP, 40); 
                  AirwayControl (CLOSE, 0); 

               */
                  presure_control_error_accu = 0;
           //MaintainPresureVentilation (peep_pres, insp_pres_filtered);
           break;
       case MAINTAIN:
           MaintainPresureVentilation (peep_pres, insp_pres_filtered);
         
           break;
      
      }
      break;

      
      case VC:
        if ( time_in_resp_cycle < (set_resp_cycle_time * 1.0/(1.0+float(ie_ratio) )) )   //float(IEratio)  * 1.0/(1.0+1.0)) 
        { 
            if ((insp_volume < vc_tv) && (current_resp_step != PLATEAU) && (insp_pres_filtered < alarm_high_pres_param) ){
              current_resp_step = INSPIRATION;
            }
            else if (current_resp_step == INSPIRATION){
              current_resp_step = PLATEAU;
              if (insp_pres_filtered >= alarm_high_pres_param)
                    alarm_high_pres = 1;
            }
            
        }
        else
        {
            if ( (current_resp_step == INSPIRATION) || (current_resp_step == PLATEAU)) {
              current_resp_step = EXPIRATION;  
              
    
            }
            if (( current_resp_step == EXPIRATION) && (insp_pres_filtered < (peep_pres+1.0) ))
            {
              current_resp_step = MAINTAIN; 
            }
        }


         lcd.setCursor(0,3);

        /* Execture breath action */
        switch (current_resp_step)
        {
          case INSPIRATION:
             // MaintainFlowVentilation (vc_tv, insp_volume);
               AirwayControl (INSP, 100); 
                presure_control_error_accu = 0;
              //  lcd.print(" >      ");
             break;
          case PLATEAU:
             //MaintainFlowVentilation (vc_tv, insp_volume);
             // MaintainPresureVentilation (peep_pres, insp_pres_filtered);
               // lcd.print(" ^      ");
              MaintainPresureVentilation (max_pres_in_cycle *0.95, insp_pres_filtered);
             break;
         case EXPIRATION:
                AirwayControl (EXP, ( insp_pres_filtered - (peep_pres+1.0) ) / 5.0 * 100.0 );
            
            /*
             if ( abs (peep_pres - insp_pres_filtered) > 3.0)
                    AirwayControl (EXP, 100);
              else
                  //AirwayControl (EXP, 40); 
                  AirwayControl (CLOSE, 0); 

             */
                  presure_control_error_accu = 0;
                  /* smooth end of exaltion */
              // lcd.print(" <      ");
             //presure_control_error_accu = 0;
             //MaintainPresureVentilation (peep_pres, insp_pres_filtered);
             break;
         case MAINTAIN:
             MaintainPresureVentilation (peep_pres, insp_pres_filtered);
            //  lcd.print(" +      ");
             break;
        }
      break;
      
      
    }


    

}

void Alarms (void)
{
     
  if (blow_active && (alarm_pc_peak_not_achieved + alarm_vc_volume_not_achieved + alarm_high_pres + alarm_min_tv + alarm_max_tv + alarm_low_pres) > 0)
      sound_request =     ERRORS;

}

void setup() 
{
  Serial.begin(9600);

  /* Display Init */
  lcd.init();
  lcd.backlight();
  lcd.print("Booting...");

  /* Pin Setting */
  inspValveServo.attach(INSP_VALVE_SERVO_PIN); 
  expValveServo.attach(EXP_VALVE_SERVO_PIN); 
  blowerServo.attach(BLOWER_SERVO_PIN);
  pinMode(INSP_PRES_SENSOR_PIN, INPUT);
  pinMode(INSP_FLOW_SENSOR_PIN, INPUT);
  pinMode(INSP_PRES_SENSOR_OSCIL_PIN, OUTPUT);
  pinMode(BLOWER_ANALOG_PIN, OUTPUT);

  
  pinMode( KP_BLOW_STBY, INPUT_PULLUP);
  pinMode( KP_CANCEL, INPUT_PULLUP);
  pinMode( KP_ENTER, INPUT_PULLUP);
  pinMode( KP_UP, INPUT_PULLUP);
  pinMode( KP_DOWN, INPUT_PULLUP);

  /* Self Check procedure */
  lcd.clear();
  lcd.print("Self check...");

  
  analogWrite ( INSP_PRES_SENSOR_OSCIL_PIN, 200);

  /* Valves Open test */


  lcd.setCursor(0,1);
  lcd.print("Valves Open");
  inspValveServo.write(insp_valve_open); 
  expValveServo.write(exp_valve_open);
  delay (2000);

  lcd.setCursor(0,1);
  lcd.print("Valves Closed");
  inspValveServo.write(insp_valve_closed); 
  expValveServo.write(exp_valve_closed);
  delay (2000);

  lcd.setCursor(0,1);
  lcd.print("Valves Open");
  inspValveServo.write(insp_valve_open); 
  expValveServo.write(exp_valve_open);
  delay (2000);

  /* Blower test */
  lcd.setCursor(0,1);
  lcd.print("Blower test");
//  for ( unsigned char i = blower_analog_min; i < blower_analog_max; i++)
//  {
//    analogWrite ( BLOWER_ANALOG_PIN, i );
//    delay (100);
//  }
//  delay (2000);
//  
//  analogWrite ( BLOWER_ANALOG_PIN, blower_analog_off );
//  delay (2000);

/* detach the servo for stand by mode */
inspValveServo.detach(); 
expValveServo.detach(); 

//  /* Tone Generator */
  analogWrite ( SPEAKER_PIN, 100 );
  delay (500);
  analogWrite ( SPEAKER_PIN,  0);
  
  lcd.clear();
  lcd.print("Ready");
  delay (1000);
  lcd.clear();
}

void loop() 
{
  /* Timing Management */
  current_time = millis ();


  UpdateAirMeasure ();
  PlaySounds();
  AquireKeypad();

  set_resp_cycle_time = 1000.0/(fr / 60.0);
  time_in_resp_cycle = current_time-last_resp_time;

  /* Update Compute */
  if ( (current_time - last_compute_loop_time) >= (COMPUTE_LOOP_PERIODE))
  {
    
    /* Variables updates and calculation */
    insp_pres_filtered = Lookup ( int (insp_pres_raw_filtered) + insp_pres_lookup_raw_measure_offset, (const int*) &insp_pres_lookup_raw[0],  (const int*) &insp_pres_lookup_presure[0], insp_pres_lookup_nbval);
    insp_flow_filtered = Lookup ( int (insp_flow_raw_filtered) + insp_flow_lookup_raw_measure_offset, (const int*) &insp_flow_lookup_raw[0],  (const int*) &insp_flow_lookup_flow[0], insp_flow_lookup_nbval); /* L/Min */
    
    if (blow_active || ((isModifyingParameter ==1) && (current_menu == SENSORS_CALIB)))
    {
      if (current_resp_step == INSPIRATION )
        insp_volume += (insp_flow_filtered/60.0) * float (COMPUTE_LOOP_PERIODE)  ; /* mL */

      if (insp_volume >=9999)
            insp_volume = 9999;
      if (insp_pres_filtered > max_pres_in_cycle)
          max_pres_in_cycle = insp_pres_filtered;

       if (insp_pres_filtered < min_pres_in_cycle)
          min_pres_in_cycle = insp_pres_filtered;
    }
    else {
      insp_volume = 0;
      max_pres_in_cycle = 0;
      min_pres_in_cycle = 0;
      insp_flow_lookup_raw_measure_offset = insp_flow_lookup_raw[0] - insp_flow_raw_filtered ;
      insp_pres_lookup_raw_measure_offset = insp_pres_lookup_raw[1] - insp_pres_raw_filtered    ;
      
    }

    /* Actions */
 
    if ((current_menu != SENSORS_CALIB))
    {
      ManageVentilation ();
     BlowerControl();
    }
    
    
    SerialOutput ();
   
    last_compute_loop_time = current_time;


 }


  /* Update HMI */ 
  if ( ((current_time - last_display_loop_time) > DISPLAY_LOOP_PERIODE) )
  {   
      ProceedKeypad();

      UpdateDisplay();

      Alarms();

      last_display_loop_time = current_time;
      keypad_pressed = NONE;

      insp_pres_raw_max = 0;
  }
  delay (1);
 
}
